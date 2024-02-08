# Installing RT kernel for Ubuntu 18 guide

Download:

[patch-5.4.26-rt17.patch.xz](https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.26-rt17.patch.xz)
from
<https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/>

[linux-5.4.26.tar.gz](https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.4.26.tar.gz)
from

<https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/>

# Ubuntu 18.04 (kernel 5.4.26) instructions

1.  Install the following necessary dependencies sudo *apt-get install
    build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev
    lsb-release libelf-dev bison flex liblz4-tool libncurses-dev
    libssl-dev* (The liblz4-tool dependency is added here since it is
    not installed by default on ubuntu 18.04)
2.  Download the regular kernel and the patch from the [Linux Kernel
    archive](https://www.kernel.org/pub/linux/kernel/projects/rt/)
3.  Verifying file integrity
4.  Unzip the Linux kernel *tar -xvf linux-5.4.26.tar*
5.  Go into the kernel folder *cd linux-5.4.26*
6.  extract the patch xz -d patch-*5.4.26*-rt17.patch.xz
7.  Apply the patch *patch -p1 \< ../patch-5.4.26-rt17.patch*
8.  Run *make menuconfig* and change the Preemption Model to *Fully
    Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)* (see [this
    stack](https://unix.stackexchange.com/questions/582075/trouble-selecting-fully-preemptible-kernel-real-time-when-configuring-compil)
    question for more info)
9.  Disable debug info *scripts/config --disable DEBUG_INFO* (Not needed
    but recommended).
10. Run *make clean*
11. Make the bzImage sudo *make -j $(nproc) bzImage*
12. Make the modules sudo *make -j $(nproc) modules*
13. Install the modules sudo *make -j $(nproc) modules_install*
14. Install the kernel sudo *make -j $(nproc) install*

tips:

- For folks wondering how to enable expert mode as suggested in the
  answer, I was compiling a 5.6.19 kernel and I wasn't able to figure
  how to select that expert mode. So I first selected embedded system
  option (General Setup -\> Embedded System) then I selected General
  Setup -\> Preemption Model -\> Fully Preemptible Kernel

- kernel and patch versions should match

- permission errors: add sudo to commands

- if you get compiling errors: check dependancies for packages needed

  - sudo apt-get install liblz4-tool
  - also fix dependancies using apt fix

main references used

<https://github.com/frankaemika/libfranka/issues/62> (unhide
instructions for ubuntu 18 at the top)

<https://stackoverflow.com/questions/51669724/install-rt-linux-patch-for-ubuntu>
(useful but didn’t work, proposes an alternative to using fakeroot which
no longer works)

<https://frankaemika.github.io/docs/installation_linux.html> (original
instructions from Franka, not working anymore)

# Commands

curl -SLO
https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.19.59.tar.xz

curl -SLO
https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.19.59.tar.sign

curl -SLO
https://www.kernel.org/pub/linux/kernel/projects/rt/4.19/older/patch-4.19.59-rt24.patch.xz

curl -SLO
https://www.kernel.org/pub/linux/kernel/projects/rt/4.19/older/patch-4.19.59-rt24.patch.sign

And decompress them with:

xz -d linux-4.19.59.tar.xz

xz -d patch-4.19.59-rt24.patch.xz

You can use *gpg2* to verify the *.tar* archives:

gpg2 --verify linux-4.19.59.tar.sign

key: 647F28654894E3BD457199BE38DBBDC86092693E

gpg2 --keyserver hkp://keys.gnupg.net --recv-keys
647F28654894E3BD457199BE38DBBDC86092693E

gpg2 --verify linux-4.19.59.tar.sign

verify patch

gpg2 --verify patch-4.19.59-rt24.patch.sign

Once you are sure the files were downloaded properly, you can extract
the source code and apply the patch:

tar xf linux-4.19.59.tar

cd linux-4.19.59

patch -p1 \< ../patch-4.19.59-rt24.patch

sudo dpkg -i ../linux-headers-4.19.59-rt24\_\*.deb
../linux-image-4.19.59-rt24\_\*.deb
