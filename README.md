# Owl-AFC-TC Klipper add-ons

Klipper extensions made for the Owl-AFC-TC automatic filament changer/toolchanger system for use in parallel with [klipper-toolchanger](https://github.com/viesturz/klipper-toolchanger) and [AFC-Klipper-Add-On](https://github.com/ArmoredTurtle/AFC-Klipper-Add-On).

## Installation

To install this plugin, run the installation script using the following command over SSH. This script will download this GitHub repository to your Raspberry Pi home directory, and symlink the files in the klippy extras folder:

```bash
wget -O - https://raw.githubusercontent.com/Eytecz/Owl-AFC-TC-Klipper/main/install.sh | bash
```

Then, add the following to your moonraker.conf to enable automatic updates:

```bash
[update_manager klipper-owl-afc-tc]
type: git_repo
channel: dev
path: ~/klipper-owl-afc-tc
origin: https://github.com/Eytecz/Owl-AFC-TC-Klipper.git
managed_services: klipper
primary_branch: main`
```

Note: If an update has new Klipper files, they will not be automatically installed into Klipper. You will need to run the install script manually:

```bash
bash ~/klipper-owl-afc-tc/install.sh
```