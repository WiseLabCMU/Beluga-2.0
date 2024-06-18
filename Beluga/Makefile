NORDIC=$(HOME)
BUILD=build
BOARD=decawave_dwm1001_dev

all: compile

.PHONY: compile
compile: $(BUILD)
	echo $(HOME)
	export BOARD=$(BOARD); \
	export LD_LIBRARY_PATH=$$LD_LIBRARY_PATH:$(NORDIC)/nordic/toolchains/2be090971e/usr/local/lib; \
	cd $(BUILD); \
	cmake ..; \
	make

$(BUILD):
	@mkdir $@

clean:
	cd $(BUILD); \
	make clean

vclean:
	$(RM) -r $(BUILD)
