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

# Valid optimization levels
VALID_OPT_LEVELS = Speed Memory Debug
# Default value for OPT_LEVEL
OPT_LEVEL ?= Debug

ifneq ($(filter $(OPT_LEVEL), $(VALID_OPT_LEVELS)), $(OPT_LEVEL))
$(error Invalid OPT_LEVEL value: $(OPT_LEVEL). Valid values are: $(VALID_OPT_LEVELS))
endif

# Define flags based on OPT_LEVEL
ifeq ($(OPT_LEVEL), Debug)
    OPT_FLAGS = -DCMAKE_BUILD_TYPE=Debug
else ifeq ($(OPT_LEVEL), Speed)
    OPT_FLAGS = -DCONFIG_SPEED_OPTIMIZATIONS=y
else ifeq ($(OPT_LEVEL), Memory)
    OPT_FLAGS = -DCONFIG_SIZE_OPTIMIZATIONS=y
endif

# Valid pristine options
VALID_PRISTINE = y yes Yes n no No
# Default value for pristine
PRISTINE ?= n

# Check if PRISTINE is valid
ifneq ($(filter $(PRISTINE), $(VALID_PRISTINE)), $(PRISTINE))
$(error Invalid PRISTINE value: $(PRISTINE). Valid values are: $(VALID_PRISTINE))
endif

ifeq ($(filter $(PRISTINE), y yes Yes), $(PRISTINE))
    PRISTINE_BUILD = --pristine
else
    PRISTINE_BUILD =
endif

all: decawave nrf21540dk nrf52dk beluga

decawave:
	@bash -c "$(ENV); \
			west build --build-dir $(PWD)/$(DECAWAVE_BUILD_DIR) \
			$(PWD)/Beluga --board decawave_dwm1001_dev --no-sysbuild $(PRISTINE_BUILD) \
			-- -DNCS_TOOLCHAIN_VERSION=NONE -DBOARD_ROOT=$(PWD) \
			$(OPT_FLAGS) -DCMAKE_MAKE_PROGRAM=ninja \
			-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
			-DDTC_OVERLAY_FILE=$(PWD)/$(DECAWAVE_OVERLAY)"

nrf21540dk:
	@bash -c "$(ENV); \
			west build --build-dir $(PWD)/$(NRF21_BUILD_DIR) \
			$(PWD)/Beluga --board nrf21540dk_nrf52840 --no-sysbuild $(PRISTINE_BUILD) \
			-- -DNCS_TOOLCHAIN_VERSION=NONE -DCMAKE_MAKE_PROGRAM=ninja \
			-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
			-DBOARD_ROOT=$(PWD) -DDTC_OVERLAY_FILE=$(PWD)/$(NRF21540_OVERLAY) \
			$(OPT_FLAGS)"

nrf52dk:
	@bash -c "$(ENV); \
    			west build --build-dir $(PWD)/$(NRF52DK_BUILD_DIR) \
    			$(PWD)/Beluga --board nrf52dk_nrf52832 --no-sysbuild $(PRISTINE_BUILD) \
    			-- -DNCS_TOOLCHAIN_VERSION=NONE -DCMAKE_MAKE_PROGRAM=ninja \
    			-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
    			-DBOARD_ROOT=$(PWD) -DDTC_OVERLAY_FILE=$(PWD)/$(NRF52832_OVERLAY) \
    			$(OPT_FLAGS)"

beluga:
	@bash -c "$(ENV); \
			west build --build-dir $(PWD)/$(BELUGA_BUILD_DIR) \
        	$(PWD)/Beluga --board beluga2 --no-sysbuild $(PRISTINE_BUILD) \
        	-- -DNCS_TOOLCHAIN_VERSION=NONE -DCMAKE_MAKE_PROGRAM=ninja \
        	-DCACHED_CONF_FILE=$(PWD)/Beluga/prj.conf \
        	-DBOARD_ROOT=$(PWD) $(OPT_FLAGS)"

format:
	@cd Beluga; \
	make format

veryclean:
	@rm -rf $(DECAWAVE_BUILD_DIR) $(NRF21_BUILD_DIR) $(NRF52DK_BUILD_DIR) $(BELUGA_BUILD_DIR)
