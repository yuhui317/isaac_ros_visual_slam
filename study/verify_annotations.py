#!/usr/bin/env python3
# 学习注释审查：比较固定 Git 基线和工作区，只读源码、解析语法，不导入上游依赖。

import argparse
import ast
import io
import json
from pathlib import Path
import re
import subprocess
import sys
import tokenize

ROOT = Path(__file__).resolve().parents[1]
CJK = re.compile(r'[\u4e00-\u9fff]')
CPP_SUFFIXES = {'.cpp', '.hpp', '.h', '.cc', '.c', '.cuh', '.cu'}


# 使用参数列表执行只读 Git，避免 shell 解释路径或源码文本。
def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])


# 读取基线的 blob 和 gitlink；未初始化的 submodule 是版本引用，不是丢失的源码文件。
def baseline_entries(baseline):
    entries = []
    for record in git('ls-tree', '-r', '-z', baseline).decode().split('\0'):
        if record:
            metadata, path = record.split('\t', 1)
            mode, kind, oid = metadata.split()
            entries.append((path, mode, kind, oid))
    return entries


# 原行必须顺序完整保留；额外行单独返回，换行风格与文件尾也不能偷偷改写。
def added_lines(before, after):
    original = before.splitlines(keepends=True)
    current = after.splitlines(keepends=True)
    cursor = 0
    additions = []
    for number, line in enumerate(current, 1):
        if cursor < len(original) and line == original[cursor]:
            cursor += 1
        else:
            additions.append((number, line))
    if cursor != len(original):
        raise ValueError(f'原始行被删除、替换或重排：baseline line {cursor + 1}')
    return additions


# 去除 Python 注释及位置相关 token；缩进、字符串与执行操作仍参与等价比较。
def python_tokens(source):
    ignored = {tokenize.COMMENT, tokenize.NL, tokenize.ENCODING, tokenize.ENDMARKER}
    return [(t.type, t.string) for t in tokenize.generate_tokens(io.StringIO(source).readline)
            if t.type not in ignored]


# 检查每个活动类/函数/嵌套定义的中文前置说明，忽略只存在于注释中的停用代码。
def python_symbols(source):
    lines = source.splitlines()
    results = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            start = min([node.lineno] + [d.lineno for d in node.decorator_list]) - 1
            preceding = []
            index = start - 1
            while index >= 0:
                line = lines[index].strip()
                if line and not line.startswith('#'):
                    break
                if line.startswith('#'):
                    preceding.append(line)
                index -= 1
            results.append({'name': node.name, 'line': node.lineno,
                            'annotated': any(CJK.search(line) for line in preceding)})
    return results


# 保守扫描 C/C++ 非注释词元：保留字符串、字符、raw string、操作符与数字，不执行预处理。
def cpp_tokens(source):
    tokens = []
    index = 0
    word = re.compile(r'[A-Za-z_\u0080-\uffff][\w\u0080-\uffff]*')
    number = re.compile(r'(?:\d|\.\d)[A-Za-z0-9_.\'\+\-]*')
    raw = re.compile(r'(?:u8|u|U|L)?R"([^ ()\\\t\r\n]{0,16})\(')
    quoted = re.compile(r'(?:u8|u|U|L)?["\']')
    operator = re.compile(r'%:%:|<=>|>>=|<<=|->\*|\.\.\.|##|::|->|\.\*|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||\+=|-=|\*=|/=|%=|\^=|&=|\|=|<:|:>|<%|%>|%:')
    while index < len(source):
        if source[index].isspace():
            index += 1
            continue
        if source.startswith('//', index):
            end = source.find('\n', index)
            index = len(source) if end < 0 else end
            continue
        if source.startswith('/*', index):
            end = source.find('*/', index + 2)
            if end < 0:
                raise ValueError('未闭合 C++ 块注释')
            index = end + 2
            continue
        match = raw.match(source, index)
        if match:
            closing = ')' + match.group(1) + '"'
            end = source.find(closing, match.end())
            if end < 0:
                raise ValueError('未闭合 C++ raw string')
            end += len(closing)
        else:
            match = quoted.match(source, index)
            if match:
                quote = source[match.end() - 1]
                end = match.end()
                while end < len(source):
                    if source[end] == '\\':
                        end += 2
                    elif source[end] == quote:
                        end += 1
                        break
                    else:
                        end += 1
                else:
                    raise ValueError('未闭合 C++ 字符串或字符')
            else:
                match = number.match(source, index) or word.match(source, index) or operator.match(source, index)
                end = match.end() if match else index + 1
        tokens.append(source[index:end])
        index = end
    return tokens


# 禁止在反斜杠续行内部插入注释；C++ 宏和 Python 显式续行需要保持原相邻关系。
def continuation_guard(source, additions, cpp=False):
    lines = source.splitlines()
    for number, line in additions:
        # C++ 先做行拼接再识别 //，新增注释自身不能吞掉后续原代码。
        if cpp and line.rstrip().endswith('\\'):
            raise ValueError(f'新增 C++ 注释以续行反斜杠结尾：line {number}')
        if number > 1:
            previous = lines[number - 2].rstrip()
            trailing = len(previous) - len(previous.rstrip('\\'))
            if (cpp and trailing) or (not cpp and trailing % 2):
                raise ValueError(f'注释插入反斜杠续行内部：line {number}')


# 仅允许注释范围中的源码变化；README 追加导航、.gitignore 只开放本次 study 目录。
def verify_file(relative, before_bytes, after_bytes, annotated):
    path = Path(relative)
    changed = before_bytes != after_bytes
    result = {'path': relative, 'changed': changed}
    if relative not in annotated and relative not in {'README.md', 'README_zh.md', '.gitignore'}:
        if changed:
            raise ValueError('范围外文件发生字节修改')
        result['protected_bytes'] = 'equal'
        return result
    before = before_bytes.decode('utf-8')
    after = after_bytes.decode('utf-8')
    additions = added_lines(before, after)
    result['added_lines'] = len(additions)
    if path.name in {'README.md', 'README_zh.md'}:
        if not after.startswith(before):
            raise ValueError('README 导航必须追加，原内容保持完整')
        return result
    if path.name == '.gitignore':
        for line_number, line in additions:
            rule = line.strip()
            if rule and not rule.startswith('#') and rule not in {'!study/', '!study/**', '!/study/', '!/study/**'}:
                raise ValueError(f'超出 study 的新增忽略规则：line {line_number}')
        return result
    prefix = '#' if path.suffix == '.py' else '//'
    if path.suffix != '.py' and path.suffix not in CPP_SUFFIXES:
        raise ValueError('注释范围包含暂未支持的源码类型')
    for line_number, line in additions:
        if line.strip() and not line.lstrip().startswith(prefix):
            raise ValueError(f'新增内容不是整行 {prefix} 注释：line {line_number}')
    if not any(line.lstrip().startswith(prefix) and CJK.search(line) for _, line in additions):
        raise ValueError('范围内源码没有新增中文说明')
    continuation_guard(after, additions, cpp=path.suffix in CPP_SUFFIXES)
    if path.suffix == '.py':
        if ast.dump(ast.parse(before), include_attributes=False) != ast.dump(ast.parse(after), include_attributes=False):
            raise ValueError('Python AST 改变')
        if python_tokens(before) != python_tokens(after):
            raise ValueError('Python 执行 token 改变')
        compile(after, relative, 'exec', dont_inherit=True)
        symbols = python_symbols(after)
        missing = [s for s in symbols if not s['annotated']]
        if missing:
            raise ValueError('定义缺少中文说明：' + ', '.join(f"{s['name']}:{s['line']}" for s in missing))
        result.update(python_ast='equal', python_executable_tokens='equal', definitions=len(symbols))
    else:
        if cpp_tokens(before) != cpp_tokens(after):
            raise ValueError('C++ 非注释词元改变')
        result['cpp_noncomment_tokens'] = 'equal'
    return result


# 遍历所有基线文件和子模块引用，汇总原行保留、语法等价与诚实的阅读覆盖范围。
def main():
    parser = argparse.ArgumentParser(description='Verify study annotations against the fixed repository baseline')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    scope = json.loads((ROOT / 'study/annotation_scope.json').read_text())
    baseline = git('rev-parse', scope['baseline'] + '^{commit}').decode().strip()
    annotated = set(scope['annotated_files'])
    entries = baseline_entries(baseline)
    original_paths = {entry[0] for entry in entries}
    failures = []
    results = []
    submodules = []
    for relative in annotated - original_paths:
        failures.append({'path': relative, 'error': '注释范围不在原始基线中'})
    for relative, mode, kind, oid in entries:
        if kind == 'commit':
            actual = git('ls-files', '--stage', '--', relative).decode().strip().split()
            if len(actual) < 2 or actual[0] != '160000' or actual[1] != oid:
                failures.append({'path': relative, 'error': 'submodule 固定引用改变'})
            else:
                submodules.append({'path': relative, 'commit': oid, 'reference': 'equal',
                                  'source_files_checked': False})
            continue
        path = ROOT / relative
        if not path.is_file() or path.is_symlink():
            failures.append({'path': relative, 'error': '原始文件丢失或被替换为符号链接'})
            continue
        try:
            before = git('show', baseline + ':' + relative)
            result = verify_file(relative, before, path.read_bytes(), annotated)
            expected_executable = mode == '100755'
            if bool(path.stat().st_mode & 0o111) != expected_executable:
                raise ValueError('文件执行权限改变')
            results.append(result)
        except (ValueError, SyntaxError, UnicodeError, subprocess.CalledProcessError) as error:
            failures.append({'path': relative, 'error': str(error)})
    self_source = Path(__file__).read_text()
    compile(self_source, __file__, 'exec', dont_inherit=True)
    if not all(s['annotated'] for s in python_symbols(self_source)):
        failures.append({'path': 'study/verify_annotations.py', 'error': '审查工具定义缺少中文说明'})
    # 新增索引工具不属于上游基线，但仍需可编译且每个活动定义有中文说明。
    study_tools = []
    for tool_path in sorted((ROOT / 'study').glob('*.py')):
        tool_source = tool_path.read_text()
        tool_relative = tool_path.relative_to(ROOT).as_posix()
        try:
            compile(tool_source, str(tool_path), 'exec', dont_inherit=True)
            tool_symbols = python_symbols(tool_source)
            if not all(symbol['annotated'] for symbol in tool_symbols):
                raise ValueError('学习工具定义缺少中文说明')
            study_tools.append({'path': tool_relative, 'syntax': 'passed', 'definitions': len(tool_symbols)})
        except (ValueError, SyntaxError, UnicodeError) as error:
            failures.append({'path': tool_relative, 'error': str(error)})
    summary = {
        'baseline': baseline,
        'branch': git('branch', '--show-current').decode().strip(),
        'original_files': sum(entry[2] == 'blob' for entry in entries),
        'verified_files': len(results),
        'gitlink_references': submodules,
        'annotated_files': len(annotated),
        'modified_original_files': sum(item['changed'] for item in results),
        'annotated_python_definitions': sum(item.get('definitions', 0) for item in results),
        'runtime_or_gpu_executed': False,
        'cpp_compilation_executed': False,
        'new_study_tools': study_tools,
        'limitations': 'C++ lexical/static check only, not preprocessing or compilation; pinned submodule contents are not checked',
        'passed': not failures,
        'failures': failures,
        'files': results,
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'files'}, ensure_ascii=False, indent=2))
    return 0 if not failures else 1


# 只有明确执行此脚本时才进行审查；读取源码或导入工具不会启动检查。
if __name__ == '__main__':
    sys.exit(main())
