# Airgap LLM Kit Builder

This tool builds a **fully offline (air-gapped) deployment kit** for running Ollama-backed LLMs with **Open WebUI** on Linux systems that have no internet access.

It is designed to run on a **connected build machine** and produce a **self-contained bundle** that can be transferred via USB / external disk to an air-gapped host.

---

## What This Tool Does

On the **build machine**, it can:

- Download all required `.deb` packages for Docker and runtime dependencies
- Download the Ollama installer
- Pull one or more Ollama models
- Optionally snapshot the Ollama model store
- Pull and export the Open WebUI Docker image
- Package everything into a single `.tgz` bundle with progress reporting

On the **air-gapped machine**, the user can:

Extract the archive.
In the scripts directory there are two scripts. 
install_airgapllm.sh:
- Installs Docker and dependencies from local `.deb` files (without internet access)
- Installs Ollama without internet access
- Load the Open WebUI Docker image
- Restore pre-pulled models (optional)

start_webui.sh
- Start Open WebUI and Ollama locally
- Navitage to localhost:8080 in a web browser.



---

## Requirements (Build Host)

- Linux (native or WSL2)
- Python 3.10+
- `sudo` access (the tool expects to be run as `root` or via `sudo -s`)
- Internet connectivity
- Sufficient disk space:
  - ~10–15 GB per large model
  - ~5 GB for Open WebUI image
  - Additional space for packaging

**Recommended**: Run from an ext4 filesystem  
(WSL `/tmp` or Linux native, not `/mnt/*`)

---

## Running the Builder

### Recommended Invocation

```bash
sudo -s <= currently important to do before you run ***TRUST***
python3 airgap_llm_builder.py

