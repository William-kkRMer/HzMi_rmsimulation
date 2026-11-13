#!/usr/bin/env bash
# 提交前必跑：校验所有被改过的 YAML（我们已经被同一个坑绊过两次）
set -u
cd "$(dirname "$0")/../.." || exit 1
files=$(git status --porcelain | awk '{print $2}' | grep -E '\.(yaml|yml|rviz)$')
[ -z "$files" ] && files=$(git diff --name-only HEAD~1 2>/dev/null | grep -E '\.(yaml|yml|rviz)$')
if [ -z "$files" ]; then echo "没有 YAML 改动"; exit 0; fi
python3 - "$files" <<'PY'
import sys, yaml
bad = 0
for f in sys.argv[1].split():
    try:
        yaml.safe_load(open(f)); print("OK  ", f)
    except Exception as e:
        bad += 1; print("BAD ", f, "->", str(e).split("\n")[0])
sys.exit(1 if bad else 0)
PY
