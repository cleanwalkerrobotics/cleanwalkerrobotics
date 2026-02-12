#!/usr/bin/env bash
# generate-pdfs.sh — Convert HTML one-pagers to PDF
#
# Usage: ./scripts/generate-pdfs.sh
#
# Prerequisites (install ONE of the following):
#   Option A (recommended): sudo apt install -y wkhtmltopdf
#   Option B: sudo apt install -y chromium-browser
#   Option C: npm install -g puppeteer
#
# Output: docs/sales/one-pagers/pdf/*.pdf

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HTML_DIR="$REPO_ROOT/docs/sales/one-pagers/html"
PDF_DIR="$REPO_ROOT/docs/sales/one-pagers/pdf"

# Create output directory
mkdir -p "$PDF_DIR"

# Detect available PDF tool
PDF_TOOL=""
if command -v wkhtmltopdf &>/dev/null; then
    PDF_TOOL="wkhtmltopdf"
elif command -v chromium &>/dev/null; then
    PDF_TOOL="chromium"
elif command -v chromium-browser &>/dev/null; then
    PDF_TOOL="chromium-browser"
elif command -v google-chrome &>/dev/null; then
    PDF_TOOL="google-chrome"
elif command -v google-chrome-stable &>/dev/null; then
    PDF_TOOL="google-chrome-stable"
fi

if [ -z "$PDF_TOOL" ]; then
    echo "ERROR: No PDF generation tool found."
    echo ""
    echo "Install one of these:"
    echo "  sudo apt install -y wkhtmltopdf      (simplest)"
    echo "  sudo apt install -y chromium-browser  (best rendering)"
    echo ""
    echo "Then re-run this script."
    exit 1
fi

echo "Using PDF tool: $PDF_TOOL"
echo "Input:  $HTML_DIR"
echo "Output: $PDF_DIR"
echo ""

# Count files
TOTAL=$(find "$HTML_DIR" -name "*.html" -type f | wc -l)
CURRENT=0
FAILED=0

for html_file in "$HTML_DIR"/*.html; do
    [ -f "$html_file" ] || continue
    CURRENT=$((CURRENT + 1))

    filename=$(basename "$html_file" .html)
    pdf_file="$PDF_DIR/${filename}.pdf"

    echo "[$CURRENT/$TOTAL] Converting: $filename"

    case "$PDF_TOOL" in
        wkhtmltopdf)
            if wkhtmltopdf \
                --page-size A4 \
                --margin-top 0 \
                --margin-right 0 \
                --margin-bottom 0 \
                --margin-left 0 \
                --enable-local-file-access \
                --no-stop-slow-scripts \
                --print-media-type \
                "$html_file" "$pdf_file" 2>/dev/null; then
                echo "  ✓ $pdf_file"
            else
                echo "  ✗ FAILED: $filename"
                FAILED=$((FAILED + 1))
            fi
            ;;
        chromium|chromium-browser|google-chrome|google-chrome-stable)
            if "$PDF_TOOL" \
                --headless \
                --disable-gpu \
                --no-sandbox \
                --print-to-pdf="$pdf_file" \
                --print-to-pdf-no-header \
                "file://$html_file" 2>/dev/null; then
                echo "  ✓ $pdf_file"
            else
                echo "  ✗ FAILED: $filename"
                FAILED=$((FAILED + 1))
            fi
            ;;
    esac
done

echo ""
echo "Done. $((CURRENT - FAILED))/$CURRENT PDFs generated successfully."
if [ "$FAILED" -gt 0 ]; then
    echo "WARNING: $FAILED file(s) failed to convert."
    exit 1
fi
