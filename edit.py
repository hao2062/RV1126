#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import argparse
import os
import re
import io

# Python 版本检测：避免在 Python 2 下运行触发语法错误
if sys.version_info[0] < 3:
    sys.stderr.write("请使用 Python 3 运行该脚本：例如 python3 ./edit.py -add <path>\n")
    sys.exit(1)

GITIGNORE = os.path.join(os.path.dirname(__file__), ".gitignore")

TRACKED_FILES_BEGIN = "# ==== TRACKED_FILES_BEGIN ===="
TRACKED_FILES_END = "# ==== TRACKED_FILES_END ===="

def read_lines(path):
    with io.open(path, "r", encoding="utf-8") as f:
        # 去除结尾换行，后续统一写回
        return [ln.rstrip("\n") for ln in f.readlines()]

def write_lines(path, lines):
    # 统一以 \n 结尾
    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

def normalize_path(p):
    p = p.strip().replace("\\", "/")
    if p.startswith("./"):
        p = p[2:]
    p = re.sub(r"^/+", "", p)          # 去前导 '/'
    p = re.sub(r"/+", "/", p)          # 折叠多斜杠
    p = p.rstrip("/")                  # 先去尾随 '/'，类型判断另行处理
    return p

def detect_type(raw):
    # 显式尾随 / 优先判定为目录
    if raw.rstrip().endswith("/"):
        return "dir", normalize_path(raw)
    n = normalize_path(raw)
    fs = os.path.join(os.getcwd(), n)
    if os.path.exists(fs):
        return ("dir" if os.path.isdir(fs) else "file"), n
    base = os.path.basename(n)
    if "." in base:
        return "file", n
    return "dir", n

def find_markers(lines):
    b = next((i for i, l in enumerate(lines) if l == TRACKED_FILES_BEGIN), None)
    e = next((i for i, l in enumerate(lines) if l == TRACKED_FILES_END), None)
    return b, e

def ensure_markers(lines):
    b, e = find_markers(lines)
    if b is not None and e is not None and b < e:
        return lines
    # 不完整则在文件末尾追加一个干净的区块
    if lines and lines[-1] != "":
        lines.append("")
    lines.append(TRACKED_FILES_BEGIN)
    lines.append("")
    lines.append(TRACKED_FILES_END)
    return lines

def split_path_levels(p):
    # 逐级累积：a -> a/b -> a/b/c
    parts = [seg for seg in p.split("/") if seg]
    acc = []
    out = []
    for seg in parts:
        acc.append(seg)
        out.append("/".join(acc))
    return out

def has_dir_block(lines, b, e, d):
    needle = "!{}/".format(d)
    for i in range(b + 1, e):
        if lines[i] == needle:
            return True
    return False

def has_file_block(lines, b, e, fpath):
    needle = "!{}".format(fpath)
    for i in range(b + 1, e):
        if lines[i] == needle:
            return True
    return False

def dir_block_end_index(lines, b, e, d):
    # 目录块末尾优先匹配 "/d/*"，否则退化为 "!d/"
    reignore = "/{}/*".format(d)
    openln = "!{}/".format(d)
    last = b
    for i in range(b + 1, e):
        if lines[i] == reignore:
            return i
    for i in range(b + 1, e):
        if lines[i] == openln:
            return i
    return last

def find_header_index(lines, b, e):
    for i in range(b + 1, e):
        if lines[i].strip() == "# path tree":
            return i
    return None

def insert_after(lines, idx, new_lines):
    return lines[: idx + 1] + new_lines + lines[idx + 1 :]

# 调整：目录块可选是否包含 "/dir/*"
def make_dir_block(d, include_reignore=True):
    block = [
        "# {}/".format(d),
        "!{}/".format(d),
    ]
    if include_reignore:
        block.append("/{}/*".format(d))
    return block

def make_file_block(fpath):
    return [
        "# {}".format(fpath),
        "!{}".format(fpath),
    ]

def list_tracked_items(lines):
    # 返回两个集合：dirs（显式打开的目录，不带尾斜杠）、files（文件），以及 reignore 集合（被关闭的目录）
    b, e = find_markers(lines)
    dirs, files, reignores = set(), set(), set()
    for i in range(b + 1, e):
        ln = lines[i].strip()
        if ln.startswith("!"):
            p = ln[1:]
            if p.endswith("/"):
                dirs.add(p[:-1])
            else:
                files.add(p)
        elif ln.startswith("/"):
            # 形如 /dir/* 标记目录被关闭
            if ln.endswith("/*"):
                lvl = ln[1:-2]  # 去掉前导 '/' 与尾部 '/*'
                if lvl:
                    reignores.add(lvl)
    return dirs, files, reignores

def has_reignore(lines, b, e, d):
    # 是否存在 "/dir/*" 关闭行
    target = "/{}/*".format(d)
    for i in range(b + 1, e):
        if lines[i] == target:
            return True
    return False

def remove_files_under_dir(lines, b, e, d):
    # 删除位于 d/ 下的所有文件条目（注释与 "!file"）
    out = []
    prefix = d + "/"
    for i, ln in enumerate(lines):
        if b < i < e:
            if ln.startswith("# ") and not ln.startswith("# /") and ln[2:].startswith(prefix):
                continue
            if ln.startswith("!") and not ln.startswith("!/") and ln[1:].startswith(prefix):
                continue
        out.append(ln)
    return out

# 调整：add_dir 增加 include_reignore 以控制是否为该目录生成 "/dir/*"
def add_dir(lines, d, include_reignore=True):
    b, e = find_markers(lines)

    # 先确保父目录存在（父目录必须包含 reignore）
    parent = os.path.dirname(d)
    if parent and parent != ".":
        lines = add_dir(lines, parent, include_reignore=True)
        b, e = find_markers(lines)
        base_idx = dir_block_end_index(lines, b, e, parent)
    else:
        header_idx = find_header_index(lines, b, e)
        base_idx = header_idx if header_idx is not None else b

    # 若要打开目录（include_reignore=False），需要：
    # 1) 删除该目录下所有已跟踪文件
    # 2) 删除该目录的关闭行（/dir/*）与可能存在的旧注释/!dir/
    if not include_reignore:
        lines = remove_files_under_dir(lines, b, e, d)
        b, e = find_markers(lines)
        if has_reignore(lines, b, e, d) or has_dir_block(lines, b, e, d):
            lines = remove_lines_for_dir(lines, b, e, d)  # 移除旧的注释/!dir//dir/*

    # 已存在 "!d/" 则视为已创建（在清理后再检查一次）
    b, e = find_markers(lines)
    if has_dir_block(lines, b, e, d) and not include_reignore:
        # 已有打开目录，无需重复插入
        return lines
    if include_reignore and has_dir_block(lines, b, e, d) and has_reignore(lines, b, e, d):
        # 已有含关闭行的目录，无需重复插入
        return lines

    # 当前目录块：目标目录根据 include_reignore 决定是否添加 "/dir/*"
    block = make_dir_block(d, include_reignore=include_reignore)
    lines = insert_after(lines, base_idx, block)
    return lines

def add_file(lines, fpath):
    b, e = find_markers(lines)
    if has_file_block(lines, b, e, fpath):
        return lines  # 已存在
    # 父目录若已“打开”（有 !parent/ 且没有 /parent/*），则不添加文件条目
    parent = os.path.dirname(fpath)
    if parent and parent != ".":
        dirs, files, reignores = list_tracked_items(lines)
        parent_opened = (parent in dirs) and (parent not in reignores)
        if parent_opened:
            return lines  # 父目录已打开，文件无需单独跟踪
        # 确保父目录链都存在（父目录需包含关闭行）
        lines = add_dir(lines, parent, include_reignore=True)
        b, e = find_markers(lines)
        base_idx = dir_block_end_index(lines, b, e, parent)
    else:
        header_idx = find_header_index(lines, b, e)
        base_idx = header_idx if header_idx is not None else b

    block = make_file_block(fpath)
    lines = insert_after(lines, base_idx, block)
    return lines

def cmd_add(path):
    if not os.path.isfile(GITIGNORE):
        print(".gitignore 不存在: {}".format(GITIGNORE), file=sys.stderr)
        sys.exit(1)
    raw = path
    kind, norm = detect_type(raw)
    lines = read_lines(GITIGNORE)
    lines = ensure_markers(lines)
    if kind == "dir":
        # 目录：若存在子文件，先清理子文件并移除关闭行，再打开整个目录
        lines = add_dir(lines, norm, include_reignore=False)
        print("added dir: {}".format(norm))
    else:
        lines = add_file(lines, norm)
        print("added file: {}".format(norm))
    write_lines(GITIGNORE, lines)

# 新增：显式添加目录
def cmd_add_dir(path):
    if not os.path.isfile(GITIGNORE):
        print(".gitignore 不存在: {}".format(GITIGNORE), file=sys.stderr)
        sys.exit(1)
    norm = normalize_path(path)
    lines = read_lines(GITIGNORE)
    lines = ensure_markers(lines)
    # 显式添加目录：若存在子文件或关闭行，清理后仅插入注释与 !dir/
    lines = add_dir(lines, norm, include_reignore=False)
    print("added dir: {}".format(norm))
    write_lines(GITIGNORE, lines)

# 新增：显式添加文件
def cmd_add_file(path):
    if not os.path.isfile(GITIGNORE):
        print(".gitignore 不存在: {}".format(GITIGNORE), file=sys.stderr)
        sys.exit(1)
    norm = normalize_path(path)
    lines = read_lines(GITIGNORE)
    lines = ensure_markers(lines)
    lines = add_file(lines, norm)
    print("added file: {}".format(norm))
    write_lines(GITIGNORE, lines)

def remove_lines_for_dir(lines, b, e, d):
    # 删除目录块相关行：注释、!d/、/d/*（各行若不存在则跳过）
    comment = "# {}/".format(d)
    openln = "!{}/".format(d)
    reignore = "/{}/*".format(d)
    out = []
    for i, ln in enumerate(lines):
        if b < i < e and ln in (comment, openln, reignore):
            continue
        out.append(ln)
    return out

def remove_lines_for_file(lines, b, e, fpath):
    # 删除文件块相关行：注释、!file
    comment = "# {}".format(fpath)
    openln = "!{}".format(fpath)
    out = []
    for i, ln in enumerate(lines):
        if b < i < e and ln in (comment, openln):
            continue
        out.append(ln)
    return out

def cmd_remove(path):
    if not os.path.isfile(GITIGNORE):
        print(".gitignore 不存在: {}".format(GITIGNORE), file=sys.stderr)
        sys.exit(1)

    norm = normalize_path(path)
    lines = read_lines(GITIGNORE)
    lines = ensure_markers(lines)
    b, e = find_markers(lines)

    # 当前已跟踪集合
    dirs, files, reignores = list_tracked_items(lines)

    # 判定目标类型
    is_dir = False
    if path.rstrip().endswith("/"):
        is_dir = True
    elif norm in files:
        is_dir = False
    elif norm in dirs:
        is_dir = True
    else:
        print("not tracked: {}".format(path))
        return

    # 先删除目标本身（目录：先删子文件，再删目录块；文件：直接删文件块）
    if is_dir:
        # 先清理该目录下所有文件条目
        lines = remove_files_under_dir(lines, b, e, norm)
        # 更新边界
        b, e = find_markers(lines)
        # 再删除目录块自身
        lines = remove_lines_for_dir(lines, b, e, norm)
    else:
        lines = remove_lines_for_file(lines, b, e, norm)

    # 更新边界与集合
    b, e = find_markers(lines)
    dirs, files, reignores = list_tracked_items(lines)

    # 构造父链（浅到深）
    def build_chain(p):
        acc, out = [], []
        for seg in [s for s in p.split("/") if s]:
            acc.append(seg); out.append("/".join(acc))
        return out

    # 起点：文件用其 dirname，目录用自身
    start_dir = os.path.dirname(norm) if not is_dir else norm
    chain = build_chain(start_dir)

    # 从最深层往上：若“该层级被关闭（存在 /lvl/*）且该层级下无任何子路径”，则删除该层级块
    for i in range(len(chain) - 1, -1, -1):
        lvl = chain[i]
        is_closed = (lvl in reignores)

        has_child = False
        if not has_child:
            for d in dirs:
                if d.startswith(lvl + "/"):
                    has_child = True
                    break
        if not has_child:
            for f in files:
                if f.startswith(lvl + "/"):
                    has_child = True
                    break

        if is_closed and (not has_child):
            lines = remove_lines_for_dir(lines, b, e, lvl)
            b, e = find_markers(lines)
            dirs, files, reignores = list_tracked_items(lines)
        else:
            break

    write_lines(GITIGNORE, lines)
    print("removed: {}".format(path))

def main():
    parser = argparse.ArgumentParser(prog="edit.py", add_help=True)
    parser.add_argument("-add", dest="add", metavar="PATH", help="自动判定并添加路径（目录或文件）")
    parser.add_argument("-add-dir", dest="add_dir", metavar="DIR", help="显式添加目录")
    parser.add_argument("-add-file", dest="add_file", metavar="FILE", help="显式添加文件")
    parser.add_argument("-remove", dest="remove", metavar="PATH", help="删除路径并清理被关闭且无子项的父层级")
    parser.add_argument("-list", dest="list", action="store_true", help="列出当前已跟踪的目录与文件")
    args = parser.parse_args()
    if args.add:
        cmd_add(args.add)
    elif args.add_dir:
        cmd_add_dir(args.add_dir)
    elif args.add_file:
        cmd_add_file(args.add_file)
    elif args.remove:
        cmd_remove(args.remove)
    elif args.list:
        cmd_list()
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()
