#!/bin/bash
# Vis move_lib.json med kun DOF-parametere (rx, ry, rz, tx, ty, tz)

FILE="${1:-../assets/moves/move_lib.json}"

jq -r 'def fmt: . * 100 | round / 100;
.moves[] |
  "[\(.index)] \(.name)",
  "  rx: [\(.params.rx.h | map("\(.amp|fmt)@\(.phase|fmt)") | join(", "))] b=\(.params.rx.bias|fmt)",
  "  ry: [\(.params.ry.h | map("\(.amp|fmt)@\(.phase|fmt)") | join(", "))] b=\(.params.ry.bias|fmt)",
  "  rz: [\(.params.rz.h | map("\(.amp|fmt)@\(.phase|fmt)") | join(", "))] b=\(.params.rz.bias|fmt)",
  "  tx: [\(.params.tx.h | map("\(.amp|fmt)@\(.phase|fmt)") | join(", "))] b=\(.params.tx.bias|fmt)",
  "  ty: [\(.params.ty.h | map("\(.amp|fmt)@\(.phase|fmt)") | join(", "))] b=\(.params.ty.bias|fmt)",
  "  tz: [\(.params.tz.h | map("\(.amp|fmt)@\(.phase|fmt)") | join(", "))] b=\(.params.tz.bias|fmt)",
  ""' "$FILE" | less
