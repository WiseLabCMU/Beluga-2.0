PYTHON_ENV=.venv
ZEPHYR_ENV=ncs/zephyr/zephyr-env.sh

# TODO: All normal builds
all: decawave

decawave:
	bash -c "source $(PYTHON_ENV)/bin/activate; source $(ZEPHYR_ENV);\
			west build --build-dir $(PWD)/cmake-build-decawave \
			$(PWD)/Beluga --board decawave_dwm1001_dev --no-sysbuild \
			-- -DNCS_TOOLCHAIN_VERSION=NONE -DBOARD_ROOT=$(PWD) \
			-DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM=ninja \
			-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
			-DDTC_OVERLAY_FILE=$(PWD)/Beluga/overlay/decawave_dwm1001_dev.overlay"
