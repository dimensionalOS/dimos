# DimOS Installer Image

## DimOS Source Repository

After booting the DimOS Ubuntu .img distribution for the first time, the DimOS source repository is fully checked out at `/home/ubuntu/dimos-git`. You can run `./bin/dev` from that directory to use the DimOS CLI.

## Generating Installer Images

Installer `.img` files can be generated using the `scripts/installer/build-installer.sh` script. See the script's usage documentation for details on how to build the image. You can find a step-by-step example of how to invoke this script in the GitHub Actions workflow file at `.github/workflows/docker.yml` in the `build-installer` job.

## Writing the Image to Disk

The `.img` file contains several baked-in partitions. When writing it to a disk, you have two options:

### Writing to the Entire Disk (MBR)

If you're writing the `.img` file to an entire disk starting at the MBR (e.g., a USB drive or dedicated disk), you can `dd` the entire `.img` file directly:

```bash
sudo dd if=dimos-desktop-installer-amd64-*.img of=/dev/sdX bs=4M status=progress
```

### Writing to a Partition

If you're writing the `.img` file to a partition (and not to the whole disk starting at the MBR), you must be careful to only `dd` the ext4 partition (the third partition, labeled "writable", GUID: 6C91986C-437F-469F-89A6-8663863ED74F) from within the `.img` file. You can use a loopback device to isolate that specific partition (the ext4 partition) and then `dd` it to your desired target partition on disk.

Here's a workflow example:

```bash
# 1. Attach the .img file as a loopback device
sudo losetup -f -P /dev/loop0 dimos-desktop-installer-amd64-*.img

# 2. List the partitions to identify the ext4 partition
lsblk /dev/loop0

# 3. Find the ext4 partition (typically loop0p3)
# You can check the filesystem type with:
sudo blkid /dev/loop0p*

# 4. Once you've identified the ext4 partition (e.g., /dev/loop0p3),
# dd it to your target partition
sudo dd if=/dev/loop0p2 of=/dev/sdXY bs=4M status=progress

# 5. Detach the loopback device when done
sudo losetup -d /dev/loop0

# 6. Run update-grub from within your main host OS so that it will detect
# the DimOS installation and add it to your disk's main GRUB menu
sudo update-grub
```

**Note:** Replace `/dev/sdXY` with your actual target partition (e.g., `/dev/sda3`). Make sure the target partition is large enough to hold the ext4 partition contents.

**Important:** The `dd` command may not properly set the correct available disk space parameter for the filesystem. After writing the partition, you should use GParted to fix the filesystem size, or use a tool other than `dd` that handles filesystem resizing automatically (such as `rsync` with appropriate options, or mounting the loopback device and using `cp`/`rsync` to copy files).

To fix the filesystem size using GParted:

1. Open GParted (install with `sudo apt install gparted` if needed)
2. Right-click on the partition you wrote to (e.g., `/dev/sda3`)
3. Select "Check" from the context menu
4. Click "Apply" to resize the filesystem to match the partition size

## Docker Images

After booting the image for the first time, the Docker images tarball is located at `/home/ubuntu/setup/artifacts/dimos-images.tar.gz`.

To load these Docker images into your local Docker image repository, run:

```bash
/home/ubuntu/setup/load-docker-images.sh
```

This script will load the Docker images from the tarball. Note that you may need to be logged in to `ghcr.io` and may require root privileges (or membership in the `docker` group) to run Docker commands.
