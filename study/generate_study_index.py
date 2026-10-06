#!/usr/bin/env python3
# 学习索引生成器：仅用固定 Git 基线与本仓 study 元数据重新生成树、行号和阅读范围。
# 不依赖 /tmp 清单或固定本机路径，不安装/导入上游库，不运行模型或修改原始源码。

import ast
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SOURCE_SUFFIXES = {'.py', '.cpp', '.hpp', '.h', '.cu', '.cuh', '.sh', '.cc', '.c'}
CPP_SUFFIXES = SOURCE_SUFFIXES - {'.py', '.sh'}
CJK = re.compile(r'[\u4e00-\u9fff]')


# 参数列表执行只读 Git 查询；不经过 shell，也不访问网络或更改分支。
def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args]).decode()


# 遍历活动定义并保留类/嵌套函数的限定名，避免同名方法在索引中混淆。
def symbols(tree, prefix=''):
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            qualified = prefix + node.name
            yield qualified, node
            yield from symbols(node, qualified + '.')
        else:
            yield from symbols(node, prefix)


# 匹配所有基线原行并返回插入行；如果源码被删改，先报错而非生成误导性注释落点。
def additions(before, after):
    original = before.splitlines(keepends=True)
    cursor = 0
    added = []
    for number, line in enumerate(after.splitlines(keepends=True), 1):
        if cursor < len(original) and line == original[cursor]:
            cursor += 1
        else:
            added.append((number, line.rstrip('\r\n')))
    if cursor != len(original):
        raise ValueError('原始行发生删改，请先审查 diff 与注释验证报告')
    return added


# 将完整文件路径转换为目录树；gitlink 标明外部引用，未注释与片段阅读分开标记。
def file_tree(paths, entries, annotated, read, depths, name):
    nested = {}
    for path in paths:
        branch = nested
        parts = path.split('/')
        for part in parts[:-1]:
            branch = branch.setdefault(part, {})
        branch[parts[-1]] = None
    lines = [name + '/']

    # 递归输出排序后的目录；相同输入产生相同树，便于检查重新生成是否改变材料。
    def render(branch, prefix='', folder=''):
        items = sorted(branch.items())
        for index, (name, children) in enumerate(items):
            last = index == len(items) - 1
            path = folder + name
            tag = ''
            suffix = '/' if children is not None else ''
            if children is None:
                if path not in entries:
                    tag = '  [新增学习材料]'
                elif entries[path][0] == 'commit':
                    tag = '  [外部 submodule @ ' + entries[path][1] + '；内部未分析]'
                elif path in annotated:
                    tag = '  [本轮中文注释]'
                elif path in read:
                    tag = '  [只读参考；未注释]'
                    if depths.get(path, '').startswith('partial:'):
                        tag = '  [片段阅读；未注释]'
                elif Path(path).suffix in SOURCE_SUFFIXES:
                    tag = '  [原样保留；未展开]'
            lines.append(prefix + ('└── ' if last else '├── ') + name + suffix + tag)
            if children is not None:
                render(children, prefix + ('    ' if last else '│   '), path + '/')

    render(nested)
    return lines


# 生成完整文件树与阅读深度表；索引不把只读片段或未下载 submodule 算成完成分析。
def tree_document(info, entries, scope):
    annotated = set(scope['annotated_files'])
    read = set(scope['read_files'])
    depths = scope.get('read_depth', {})
    new_files = {p.relative_to(ROOT).as_posix() for p in (ROOT / 'study').rglob('*')
                 if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    new_files |= {'study/code_tree.md', 'study/code_symbols.md', 'study/annotation_validation.json'}
    tree = file_tree(sorted(set(entries) | new_files), entries, annotated, read, depths, info['name'])
    doc = [f'# {info["name"]} · 完整代码树', '',
           '[初步分析](analysis.md) · [源码定位](code_symbols.md) · [范围](annotation_scope.json) · [验证](annotation_validation.json)', '',
           f'固定基线 `{info["baseline"]}`，来源 `{info["source_branch"]}`，学习分支 `{info["study_branch"]}`。', '',
           f'{info["original_files"]} 个基线文件、{len(info["gitlinks"])} 个 gitlink；本轮注释 {len(annotated)} 个原始文件。树不包含 `.git` 或缓存，固定基线自带的资源仍完整列出。', '',
           '```text', *tree, '```', '', '## 本轮阅读入口', '',
           '| 文件 | 范围 | 阅读深度 |', '|---|---|---|']
    for path in sorted(read):
        status = '中文注释' if path in annotated else '只读参考，未注释'
        depth = depths.get(path, '范围已登记；未单独标注深度').replace('|', '\\|')
        doc.append(f'| [{path}](../{path}) | {status} | {depth} |')
    doc += ['', '目录树和行号是定位材料，不代表所有代码已理解或运行。子模块只保留固定引用，内部文件不在本轮范围。', '',
            '重新生成：`python study/generate_study_index.py`。再运行 `python study/verify_annotations.py --report study/annotation_validation.json` 审查源码保留情况；两个工具均不运行上游项目。', '']
    return '\n'.join(doc)


# 索引全仓 Python 活动定义；C++ 只列本轮注释对应源码位置，不冒充完整符号解析。
def symbol_document(info, entries, scope):
    annotated = set(scope['annotated_files'])
    doc = [f'# {info["name"]} · 源码定位索引', '', '[初步分析](analysis.md) · [代码树](code_tree.md)', '',
           '行号对应当前工作区。Python AST 索引全部活动定义；只存在于注释中的停用实现不计入定义。未展开文件明确标注，生成索引不代表完成阅读。', '']
    python_count = definition_count = 0
    for path, (kind, _) in sorted(entries.items()):
        if kind != 'blob' or not path.endswith('.py'):
            continue
        python_count += 1
        items = list(symbols(ast.parse((ROOT / path).read_text())))
        definition_count += len(items)
        status = '本轮中文注释' if path in annotated else '原样保留，仅生成索引'
        doc += [f'## {path}', '', f'范围：{status}；{len(items)} 个定义。', '']
        if not items:
            doc += [f'[模块入口](../{path})：没有活动类/函数定义。', '']
            continue
        doc += ['| 符号 | 类型 | 当前行号 |', '|---|---|---|']
        for name, node in items:
            kind_name = 'class' if isinstance(node, ast.ClassDef) else ('async function' if isinstance(node, ast.AsyncFunctionDef) else 'function/method')
            doc.append(f'| `{name}` | {kind_name} | [{node.lineno}](../{path}#L{node.lineno}) |')
        doc.append('')
    doc += ['## C/C++ 注释落点', '', '只列本轮中文注释对应的下一条源码行；不是全仓 C++ 符号解析，不覆盖子模块。', '',
            '| 文件 / 行号 | 中文说明 |', '|---|---|']
    cpp_count = 0
    for path in sorted(annotated):
        if Path(path).suffix not in CPP_SUFFIXES:
            continue
        source = (ROOT / path).read_text()
        lines = source.splitlines()
        before = git('show', info['baseline'] + ':' + path)
        for number, line in additions(before, source):
            if not line.lstrip().startswith('//') or not CJK.search(line):
                continue
            next_line = number + 1
            while next_line <= len(lines) and (not lines[next_line - 1].strip() or lines[next_line - 1].lstrip().startswith('//')):
                next_line += 1
            if next_line > len(lines):
                next_line = number
            cpp_count += 1
            note = line.lstrip()[2:].strip().replace('|', '\\|')
            doc.append(f'| [{path}:{next_line}](../{path}#L{next_line}) | {note} |')
    if not cpp_count:
        doc.append('| — | 本轮未注释 C/C++ 文件 |')
    doc += ['', f'原始 Python：{python_count} 文件、{definition_count} 活动定义；C/C++：{cpp_count} 注释落点。', '',
            '新增学习工具不算上游符号，其完整中文说明在工具源码中。重新生成使用 [generate_study_index.py](generate_study_index.py)；静态验证使用 [verify_annotations.py](verify_annotations.py)。', '']
    return '\n'.join(doc), {'python_files': python_count, 'python_symbols': definition_count, 'cpp_comment_anchors': cpp_count}


# 固定基线核对后生成学习文档；任何范围路径不在本仓基线中都会立即报错。
def main():
    scope = json.loads((ROOT / 'study/annotation_scope.json').read_text())
    info = json.loads((ROOT / 'study/repository.json').read_text())
    baseline = git('rev-parse', scope['baseline'] + '^{commit}').strip()
    if baseline != info['baseline']:
        raise ValueError('scope 与 repository 的基线不同，请先核对版本')
    entries = {}
    for record in git('ls-tree', '-r', '-z', baseline).split('\0'):
        if record:
            metadata, path = record.split('\t', 1)
            _, kind, oid = metadata.split()
            entries[path] = (kind, oid)
    outside = (set(scope['annotated_files']) | set(scope['read_files'])) - set(entries)
    if outside:
        raise ValueError('范围路径未出现在基线：' + ', '.join(sorted(outside)))
    info.update(annotated_files=len(scope['annotated_files']), read_files=len(scope['read_files']),
                original_files=sum(kind == 'blob' for kind, _ in entries.values()),
                gitlinks=[{'path': path, 'commit': oid} for path, (kind, oid) in entries.items() if kind == 'commit'])
    symbols_doc, counts = symbol_document(info, entries, scope)
    (ROOT / 'study/repository.json').write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n')
    (ROOT / 'study/code_tree.md').write_text(tree_document(info, entries, scope))
    (ROOT / 'study/code_symbols.md').write_text(symbols_doc)
    print(json.dumps({'repository': info['name'], 'original_files': info['original_files'],
                      'annotated_files': info['annotated_files'], **counts}, ensure_ascii=False))


# 显式执行时才写入学习材料；导入此工具不会自动生成文件。
if __name__ == '__main__':
    main()
