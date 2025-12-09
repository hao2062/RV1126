#!/usr/bin/env bash
set -euo pipefail

GITIGNORE="./.gitignore"

TRACKED_DIRS_BEGIN="# ==== TRACKED_DIRS_BEGIN ===="
TRACKED_DIRS_END="# ==== TRACKED_DIRS_END ===="
TRACKED_FILES_BEGIN="# ==== TRACKED_FILES_BEGIN ===="
TRACKED_FILES_END="# ==== TRACKED_FILES_END ===="

usage() {
  echo "用法:"
  echo "  $0 add <path>            放开目录或文件（自动判定）"
  echo "  $0 add-dir <dir>         放开目录"
  echo "  $0 add-file <file>       放开单个文件"
  echo "  $0 remove <path>         删除放开项（目录或文件）"
  echo "  $0 list                  列出当前放开项"
  echo "  $0 validate              检查重复规则"
  exit 1
}

ensure_markers() {
  if ! grep -q "$TRACKED_DIRS_BEGIN" "$GITIGNORE"; then
    cat >> "$GITIGNORE" <<EOF

$TRACKED_DIRS_BEGIN
$TRACKED_DIRS_END

$TRACKED_FILES_BEGIN
$TRACKED_FILES_END
EOF
  fi
}

# 将一段文本插入到标记之间
insert_between_markers() {
  local begin="$1"; local end="$2"; local payload="$3"
  awk -v begin="$begin" -v end="$end" -v payload="$payload" '
    BEGIN {printed=0}
    {
      print $0
      if ($0 == begin && printed == 0) {
        print payload
        printed=1
      }
    }
  ' "$GITIGNORE" > "$GITIGNORE.tmp" && mv "$GITIGNORE.tmp" "$GITIGNORE"
}

# 删除标记段中的包含关键字的行块（按起止注释）
delete_block_by_keyword() {
  local begin="$1"; local end="$2"; local keyword="$3"
  awk -v begin="$begin" -v end="$end" -v keyword="$keyword" '
    BEGIN {in=0; skip=0}
    {
      if ($0 == begin) {in=1; print; next}
      if ($0 == end)   {in=0; skip=0; print; next}
      if (in) {
        if (index($0, keyword) != 0) {skip=1}
        if (!skip) print
        next
      }
      print
    }
  ' "$GITIGNORE" > "$GITIGNORE.tmp" && mv "$GITIGNORE.tmp" "$GITIGNORE"
}

# 生成逐级路径数组
split_path() {
  local path="$1"
  IFS='/' read -r -a parts <<< "$path"
  local acc=""
  for p in "${parts[@]}"; do
    if [[ -z "$acc" ]]; then acc="$p"; else acc="$acc/$p"; fi
    echo "$acc"
  done
}

# 规范化路径：去掉前导'./'，折叠多余斜杠，去尾部斜杠
normalize_path() {
  local p="$1"
  # 去掉前导 ./ 和重复斜杠
  p="${p#./}"
  # 新增：去掉一个或多个前导斜杠，统一按仓库根相对路径生成规则
  p="$(echo "$p" | sed -E 's#^/+##')"
  # 去掉结尾斜杠（但根 '/' 不处理）
  [[ "$p" != "/" ]] && p="${p%/}"
  # 去掉连续斜杠
  p="$(echo "$p" | sed -E 's#/+#/#g')"
  echo "$p"
}

# 检查是否已存在 BEGIN 块（兼容 awk 实现）
exists_block() {
  local begin="$1"; local end="$2"; local keyword="$3"
  awk -v b="$begin" -v e="$end" '
    $0==b {in=1; next}
    $0==e {in=0}
    in    {print}
  ' "$GITIGNORE" | grep -Fq "$keyword"
}

gen_dir_rules() {
  local dir="$(normalize_path "$1")"
  local rules=""
  local last=""
  # 找到最后一级
  while read -r level; do last="$level"; done < <(split_path "$dir")
  # 重新遍历逐级生成规则，中间层级：!<level>/ 与 /<level>/*；最后一级仅 !<level>/
  while read -r level; do
    if [[ "$level" == "$last" ]]; then
      rules+="!${level}/"$'\n'
    else
      rules+="!${level}/"$'\n'
      rules+="/${level}/*"$'\n'
    fi
  done < <(split_path "$dir")
  echo "$rules"
}

gen_file_rules() {
  local file="$(normalize_path "$1")"
  local dir="$(dirname "$file")"
  local rules=""
  # 逐级打开父目录（带尾斜杠）
  while read -r level; do
    rules+="!${level}/"$'\n'
    rules+="/${level}/*"$'\n'
  done < <(split_path "$(normalize_path "$dir")")
  # 打开具体文件（不加尾斜杠）
  rules+="!${file}"$'\n'
  echo "$rules"
}

add_dir() {
  local dir="$(normalize_path "$1")"
  ensure_markers
  local key="BEGIN DIR: $dir"
  if exists_block "$TRACKED_DIRS_BEGIN" "$TRACKED_DIRS_END" "$key"; then
    echo "已存在目录放开项: $dir"
    return 0
  fi
  # 保证 END 行独立且块末尾换行
  local payload="# ${key}"$'\n'"$(gen_dir_rules "$dir")"$'\n'"# END DIR: $dir"$'\n'
  insert_between_markers "$TRACKED_DIRS_BEGIN" "$TRACKED_DIRS_END" "$payload"
  echo "已放开目录: $dir"
}

add_file() {
  local file="$(normalize_path "$1")"
  ensure_markers
  local key="BEGIN FILE: $file"
  if exists_block "$TRACKED_FILES_BEGIN" "$TRACKED_FILES_END" "$key"; then
    echo "已存在文件放开项: $file"
    return 0
  fi
  # 保证 END 行独立且块末尾换行
  local payload="# ${key}"$'\n'"$(gen_file_rules "$file")"$'\n'"# END FILE: $file"$'\n'
  insert_between_markers "$TRACKED_FILES_BEGIN" "$TRACKED_FILES_END" "$payload"
  echo "已放开文件: $file"
}

# 自动判定类型：dir 或 file
detect_type() {
  local raw="$1"
  local path="$(normalize_path "$raw")"
  # 明确尾随斜杠视为目录
  if [[ "$raw" == */ ]]; then
    echo "dir"; return 0
  fi
  # 文件系统存在时按真实类型判断
  if [[ -e "$path" ]]; then
    if [[ -d "$path" ]]; then echo "dir"; else echo "file"; fi
    return 0
  fi
  # 不存在时，按最后一段是否包含 '.' 猜测：有扩展名 -> file，否则 dir
  local base="${path##*/}"
  if [[ "$base" == *.* ]]; then echo "file"; else echo "dir"; fi
}

add_any() {
  local input="$1"
  local type
  type="$(detect_type "$input")"
  if [[ "$type" == "dir" ]]; then
    add_dir "$input"
  else
    add_file "$input"
  fi
}

remove_path() {
  local path="$(normalize_path "$1")"
  ensure_markers
  delete_block_by_keyword "$TRACKED_DIRS_BEGIN" "$TRACKED_DIRS_END" "BEGIN DIR: $path"
  delete_block_by_keyword "$TRACKED_FILES_BEGIN" "$TRACKED_FILES_END" "BEGIN FILE: $path"
  echo "已尝试删除: $path"
}

list_items() {
  echo "放开目录："
  awk -v b="$TRACKED_DIRS_BEGIN" -v e="$TRACKED_DIRS_END" '
    $0==b{in=1; next} $0==e{in=0} in && /# BEGIN DIR:/ {sub(/^# BEGIN DIR: /,"- /"); print}
  ' "$GITIGNORE"
  echo "放开文件："
  awk -v b="$TRACKED_FILES_BEGIN" -v e="$TRACKED_FILES_END" '
    $0==b{in=1; next} $0==e{in=0} in && /# BEGIN FILE:/ {sub(/^# BEGIN FILE: /,"- /"); print}
  ' "$GITIGNORE"
}

validate() {
  echo "检查重复/冲突："
  {
    awk -v b="$TRACKED_DIRS_BEGIN" -v e="$TRACKED_DIRS_END" '
      $0==b{in=1; next} $0==e{in=0} in && /# BEGIN DIR:/ {sub(/^# BEGIN DIR: /,"DIR "); print}
    ' "$GITIGNORE"
    awk -v b="$TRACKED_FILES_BEGIN" -v e="$TRACKED_FILES_END" '
      $0==b{in=1; next} $0==e{in=0} in && /# BEGIN FILE:/ {sub(/^# BEGIN FILE: /,"FILE "); print}
    ' "$GITIGNORE"
  } | sort | uniq -d || true
  echo "提示：如存在重复条目，请 remove 后重新 add。"
}

main() {
  [[ $# -lt 1 ]] && usage
  case "$1" in
    add)
      [[ $# -ne 2 ]] && usage
      add_any "$2"
      ;;
    add-dir)
      [[ $# -ne 2 ]] && usage
      add_dir "$2"
      ;;
    add-file)
      [[ $# -ne 2 ]] && usage
      add_file "$2"
      ;;
    remove)
      [[ $# -ne 2 ]] && usage
      remove_path "$2"
      ;;
    list)
      list_items
      ;;
    validate)
      validate
      ;;
    *)
      usage
      ;;
  esac
}

main "$@"