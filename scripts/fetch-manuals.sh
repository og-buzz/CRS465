#!/usr/bin/env bash
# Download the vendor manuals listed in docs/manuals.tsv (they are not committed),
# verify each against its sha256, and extract searchable text into docs/text/.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
manuals="$repo/docs/manuals"; text="$repo/docs/text"
mkdir -p "$text"
fail=0
while IFS=$'\t' read -r sha path url; do
  [[ "$sha" == \#* || -z "$sha" ]] && continue
  dest="$manuals/$path"
  if [[ ! -f "$dest" ]]; then
    mkdir -p "$(dirname "$dest")"
    echo "fetching $path"
    curl -fsSL --max-time 300 -A 'Mozilla/5.0 (X11; Linux x86_64) curl' -o "$dest.part" "$url" && mv "$dest.part" "$dest" \
      || { echo "  DOWNLOAD FAILED: $url" >&2; rm -f "$dest.part"; fail=1; continue; }
  fi
  if [[ "$(sha256sum "$dest" | cut -d' ' -f1)" != "$sha" ]]; then
    echo "  SHA256 MISMATCH: $path (source may have changed the file)" >&2; fail=1; continue
  fi
  out="$text/$(basename "${path%.pdf}").txt"
  if [[ ! -f "$out" ]]; then
    if command -v pdftotext >/dev/null; then pdftotext -layout "$dest" "$out"
    else echo "  pdftotext not installed (apt install poppler-utils); skipping text for $path" >&2; fi
  fi
  echo "ok  $path"
done < "$repo/docs/manuals.tsv"
exit "$fail"
