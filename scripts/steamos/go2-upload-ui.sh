#!/usr/bin/env bash
# "Upload recordings" icon: the upload window, on the venv's Python inside the dimos container.
export DISPLAY="${DISPLAY:-:0}"
exec distrobox enter dimos -- bash -c "cd ~/dimos/dimensional-applications && .venv/bin/python ~/go2-upload-ui.py" >> ~/go2-app.log 2>&1
