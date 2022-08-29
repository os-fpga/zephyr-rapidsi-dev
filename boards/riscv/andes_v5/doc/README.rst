Code structure of ORCA board
#############################

orca----|
	|-- board.cmake
	|-- doc
	|   |-- index.rst
	|   |-- README.rst
	|
	|-- Kconfig.board
	|-- Kconfig.defconfig
	|-- orca_ae350_defconfig
	|-- orca_ae350.dts
	|-- orca_ae350.yaml


Building procedure
==================

	# Setup build envoronment
	cd ${ZEPHYR_BASE}
	source zephyr-env.sh

	# Specify the location of the custom toolchain
	export ZEPHYR_TOOLCHAIN_VARIANT='cross-compile'
	export CROSS_COMPILE=${CUSTOM_TOOL_CHAIN_PATH}/riscv32-elf-'

	# Zephyr only support out-of-tree build, so create a build folder
	# Take sample "sychronization" as an example
	cd samples/synchronization/
	mkdir -p build
	cd build
 
	# Generate Makefile by using cmake command
	# Now we support orca_ae250/orca_ae350 boards
	cmake -DEXTRA_LDFLAGS="-fuse-ld=bfd" -G "Unix Makefiles" -DBOARD=${BOARD_NAME} ..

	# Configure the kernel/HW by KConfig interface
	make menuconfig

	# Build the sample program
	make

	# Finally, it will generate the ELF file "build/zephyr/zephyr.elf"
	
Sanity-check
============

	# Enter the root folder of zephyr cource code
	cd ${ZEPHYR_BASE}

	# Execute the script to generate the kernel test cases for specified board
	# Now we support orca_ae250/orca_ae350 boards
	./scripts/sanitycheck -v -p ${BOARD_NAME} -T tests/kernel -x=EXTRA_LDFLAGS="-fuse-ld=bfd"


Note
====
	For building sample application:
	- We need config at least double default stack size of main thread/test function thread if we 
	  use v5f/v5d toolchain Option are located in :
		-	"General Kernel Options"--->"Size of stack for initialization and main thread"
		-	"Testing"--->"Test function thread stack size"

	For building sanity check test cases:
	- Configure double size of default setting before executing the scripts:
		-	MAIN_STACK_SIZE if ZTEST: 512-->1024
		-	TEST_EXTRA_STACKSIZE: 0 --> 512
