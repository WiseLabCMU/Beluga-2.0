PYTHON_ENV=.venv
ZEPHYR_ENV=ncs/zephyr/zephyr-env.sh
ENV=source $(PYTHON_ENV)/bin/activate; source $(ZEPHYR_ENV)

OVERLAY_DIR=Beluga/overlay
DECAWAVE_OVERLAY=$(OVERLAY_DIR)/decawave_dwm1001_dev.overlay
NRF21540_OVERLAY=$(OVERLAY_DIR)/nrf21540dk_nrf52840.overlay
NRF52832_OVERLAY=$(OVERLAY_DIR)/nrf52dk_nrf52832.overlay

DECAWAVE_BUILD_DIR=cmake-build-decawave
NRF21_BUILD_DIR=cmake-build-nrf21540dk_nrf52840
NRF52DK_BUILD_DIR=cmake-build-pca10040
BELUGA_BUILD_DIR=cmake-build-beluga

OPT_LEVEL=Debug

# TODO: All normal builds
all: decawave nrf21540dk nrf52dk beluga

decawave:
	bash -c "$(ENV); \
			west build --build-dir $(PWD)/$(DECAWAVE_BUILD_DIR) \
			$(PWD)/Beluga --board decawave_dwm1001_dev --no-sysbuild \
			-- -DNCS_TOOLCHAIN_VERSION=NONE -DBOARD_ROOT=$(PWD) \
			-DCMAKE_BUILD_TYPE=$(OPT_LEVEL) -DCMAKE_MAKE_PROGRAM=ninja \
			-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
			-DDTC_OVERLAY_FILE=$(PWD)/$(DECAWAVE_OVERLAY)"

nrf21540dk:
	bash -c "$(ENV); \
			west build --build-dir $(PWD)/$(NRF21_BUILD_DIR) \
			$(PWD)/Beluga --board nrf21540dk_nrf52840 --no-sysbuild \
			-- -DNCS_TOOLCHAIN_VERSION=NONE -DCMAKE_MAKE_PROGRAM=ninja \
			-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
			-DBOARD_ROOT=$(PWD) -DDTC_OVERLAY_FILE=$(PWD)/$(NRF21540_OVERLAY) \
			-DCMAKE_BUILD_TYPE=$(OPT_LEVEL)"

nrf52dk:
	bash -c "$(ENV); \
    			west build --build-dir $(PWD)/$(NRF52DK_BUILD_DIR) \
    			$(PWD)/Beluga --board nrf52dk_nrf52832 --no-sysbuild \
    			-- -DNCS_TOOLCHAIN_VERSION=NONE -DCMAKE_MAKE_PROGRAM=ninja \
    			-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
    			-DBOARD_ROOT=$(PWD) -DDTC_OVERLAY_FILE=$(PWD)/$(NRF52832_OVERLAY) \
    			-DCMAKE_BUILD_TYPE=$(OPT_LEVEL)"

beluga:
	bash -c "$(ENV); \
			west build --build-dir $(PWD)/$(BELUGA_BUILD_DIR) \
        	$(PWD)/Beluga --board beluga2 --no-sysbuild \
        	-- -DNCS_TOOLCHAIN_VERSION=NONE -DCMAKE_MAKE_PROGRAM=ninja \
        	-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
        	-DBOARD_ROOT=$(PWD) -DCMAKE_BUILD_TYPE=$(OPT_LEVEL)"

veryclean:
	rm -rf $(DECAWAVE_BUILD_DIR) $(NRF21_BUILD_DIR) $(NRF52DK_BUILD_DIR) $(BELUGA_BUILD_DIR)
