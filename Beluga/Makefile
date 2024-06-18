NORDIC=$(HOME)
BUILD=build
BOARD=
BOARD_CONFIG=decawave_dwm1001_dev

ifneq (, $(BOARD))
	BOARD_CONFIG=$(BOARD)
endif

all: compile

.PHONY: compile
compile: $(BUILD)
	echo $(BOARD_CONFIG); \
	export BOARD=$(BOARD_CONFIG); \
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
