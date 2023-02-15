scp ./arch/x86_64/boot/bzImage nao@$1:/home/nao/boot/bzImagenew
scp ./System.map nao@$1:/home/nao/boot/System.map.new
scp ./.config nao@$1:/home/nao/boot/config

scp ./drivers/gpu/drm/drm_kms_helper.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/gpu/drm/
scp ./drivers/gpu/drm/drm.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/gpu/drm/
scp ./drivers/gpu/drm/i915/i915.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/gpu/drm/i915
scp ./drivers/video/backlight/backlight.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/video/backlight

scp ./drivers/video/backlight/generic_bl.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/video/backlight
scp ./drivers/video/backlight/lcd.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/video/backlight
scp ./drivers/char/agp/intel-gtt.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/char/agp/
scp ./drivers/acpi/video.ko nao@$1:/home/nao/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/acpi/

