#!/usr/bin/env bash
# Headless LaTeX compile via Docker TeXLive - no local TeX install needed.
# Usage: ./build.sh [main-tex-basename]   (default: ProbRobPaperTemplateWS2022)
set -euo pipefail
MAIN="${1:-ProbRobPaperTemplateWS2022}"
DIR="$(cd "$(dirname "$0")" && pwd)"
docker run --rm \
  -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$DIR":/work -w /work texlive/texlive:latest \
  latexmk -pdf -interaction=nonstopmode -halt-on-error "$MAIN.tex"
