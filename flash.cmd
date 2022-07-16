cargo build --release
elf2uf2-rs .\target\thumbv6m-none-eabi\release\rp2040-project-template
copy .\target\thumbv6m-none-eabi\release\rp2040-project-template.uf2 e: