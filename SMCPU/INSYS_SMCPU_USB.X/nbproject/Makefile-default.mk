#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../Source/USB_STACK/usb_descriptors.c ../Source/USB_STACK/usb_device.c ../Source/USB_STACK/USB_DRV.c ../Source/USB_STACK/usb_function_cdc.c ../Source/USB_STACK/usb_hal_pic24.c ../Source/COMM_DAC.c ../Source/COMM_SMC.c ../Source/CPU_SM_MAIN.c ../Source/CRC16.c ../Source/CRC32.c ../Source/DRV_A2D.c ../Source/DRV_I2C.c ../Source/DRV_MEM.c ../Source/DRV_RTC.c ../Source/EEPROM.c ../Source/EVENTS.c ../Source/comm_host.c ../Source/command_proc.c ../Source/DRV_GLCD_SPI.c ../Source/COMM_GSM.c ../Config_bit_setting.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1669077864/usb_descriptors.o ${OBJECTDIR}/_ext/1669077864/usb_device.o ${OBJECTDIR}/_ext/1669077864/USB_DRV.o ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o ${OBJECTDIR}/_ext/1728301206/CRC16.o ${OBJECTDIR}/_ext/1728301206/CRC32.o ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o ${OBJECTDIR}/_ext/1728301206/EEPROM.o ${OBJECTDIR}/_ext/1728301206/EVENTS.o ${OBJECTDIR}/_ext/1728301206/comm_host.o ${OBJECTDIR}/_ext/1728301206/command_proc.o ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o ${OBJECTDIR}/_ext/1472/Config_bit_setting.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1669077864/usb_descriptors.o.d ${OBJECTDIR}/_ext/1669077864/usb_device.o.d ${OBJECTDIR}/_ext/1669077864/USB_DRV.o.d ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o.d ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o.d ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o.d ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o.d ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o.d ${OBJECTDIR}/_ext/1728301206/CRC16.o.d ${OBJECTDIR}/_ext/1728301206/CRC32.o.d ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o.d ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o.d ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o.d ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o.d ${OBJECTDIR}/_ext/1728301206/EEPROM.o.d ${OBJECTDIR}/_ext/1728301206/EVENTS.o.d ${OBJECTDIR}/_ext/1728301206/comm_host.o.d ${OBJECTDIR}/_ext/1728301206/command_proc.o.d ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o.d ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o.d ${OBJECTDIR}/_ext/1472/Config_bit_setting.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1669077864/usb_descriptors.o ${OBJECTDIR}/_ext/1669077864/usb_device.o ${OBJECTDIR}/_ext/1669077864/USB_DRV.o ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o ${OBJECTDIR}/_ext/1728301206/CRC16.o ${OBJECTDIR}/_ext/1728301206/CRC32.o ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o ${OBJECTDIR}/_ext/1728301206/EEPROM.o ${OBJECTDIR}/_ext/1728301206/EVENTS.o ${OBJECTDIR}/_ext/1728301206/comm_host.o ${OBJECTDIR}/_ext/1728301206/command_proc.o ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o ${OBJECTDIR}/_ext/1472/Config_bit_setting.o

# Source Files
SOURCEFILES=../Source/USB_STACK/usb_descriptors.c ../Source/USB_STACK/usb_device.c ../Source/USB_STACK/USB_DRV.c ../Source/USB_STACK/usb_function_cdc.c ../Source/USB_STACK/usb_hal_pic24.c ../Source/COMM_DAC.c ../Source/COMM_SMC.c ../Source/CPU_SM_MAIN.c ../Source/CRC16.c ../Source/CRC32.c ../Source/DRV_A2D.c ../Source/DRV_I2C.c ../Source/DRV_MEM.c ../Source/DRV_RTC.c ../Source/EEPROM.c ../Source/EVENTS.c ../Source/comm_host.c ../Source/command_proc.c ../Source/DRV_GLCD_SPI.c ../Source/COMM_GSM.c ../Config_bit_setting.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ256GB210
MP_LINKER_FILE_OPTION=,--script="linker_script.gld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1669077864/usb_descriptors.o: ../Source/USB_STACK/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_descriptors.c  -o ${OBJECTDIR}/_ext/1669077864/usb_descriptors.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_descriptors.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_descriptors.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/usb_device.o: ../Source/USB_STACK/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_device.c  -o ${OBJECTDIR}/_ext/1669077864/usb_device.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_device.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_device.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/USB_DRV.o: ../Source/USB_STACK/USB_DRV.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/USB_DRV.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/USB_DRV.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/USB_DRV.c  -o ${OBJECTDIR}/_ext/1669077864/USB_DRV.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/USB_DRV.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/USB_DRV.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o: ../Source/USB_STACK/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_function_cdc.c  -o ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o: ../Source/USB_STACK/usb_hal_pic24.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_hal_pic24.c  -o ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/COMM_DAC.o: ../Source/COMM_DAC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/COMM_DAC.c  -o ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/COMM_DAC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/COMM_DAC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/COMM_SMC.o: ../Source/COMM_SMC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/COMM_SMC.c  -o ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/COMM_SMC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/COMM_SMC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o: ../Source/CPU_SM_MAIN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/CPU_SM_MAIN.c  -o ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/CRC16.o: ../Source/CRC16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC16.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/CRC16.c  -o ${OBJECTDIR}/_ext/1728301206/CRC16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/CRC16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/CRC16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/CRC32.o: ../Source/CRC32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC32.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/CRC32.c  -o ${OBJECTDIR}/_ext/1728301206/CRC32.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/CRC32.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/CRC32.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_A2D.o: ../Source/DRV_A2D.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_A2D.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_A2D.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_A2D.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_I2C.o: ../Source/DRV_I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_I2C.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_I2C.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_I2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_MEM.o: ../Source/DRV_MEM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_MEM.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_MEM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_MEM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_RTC.o: ../Source/DRV_RTC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_RTC.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_RTC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_RTC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/EEPROM.o: ../Source/EEPROM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EEPROM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EEPROM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/EEPROM.c  -o ${OBJECTDIR}/_ext/1728301206/EEPROM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/EEPROM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/EEPROM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/EVENTS.o: ../Source/EVENTS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EVENTS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EVENTS.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/EVENTS.c  -o ${OBJECTDIR}/_ext/1728301206/EVENTS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/EVENTS.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/EVENTS.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/comm_host.o: ../Source/comm_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/comm_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/comm_host.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/comm_host.c  -o ${OBJECTDIR}/_ext/1728301206/comm_host.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/comm_host.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/comm_host.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/command_proc.o: ../Source/command_proc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/command_proc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/command_proc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/command_proc.c  -o ${OBJECTDIR}/_ext/1728301206/command_proc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/command_proc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/command_proc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o: ../Source/DRV_GLCD_SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_GLCD_SPI.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/COMM_GSM.o: ../Source/COMM_GSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/COMM_GSM.c  -o ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/COMM_GSM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/COMM_GSM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/Config_bit_setting.o: ../Config_bit_setting.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/Config_bit_setting.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Config_bit_setting.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Config_bit_setting.c  -o ${OBJECTDIR}/_ext/1472/Config_bit_setting.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/Config_bit_setting.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Config_bit_setting.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1669077864/usb_descriptors.o: ../Source/USB_STACK/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_descriptors.c  -o ${OBJECTDIR}/_ext/1669077864/usb_descriptors.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_descriptors.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_descriptors.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/usb_device.o: ../Source/USB_STACK/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_device.c  -o ${OBJECTDIR}/_ext/1669077864/usb_device.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_device.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_device.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/USB_DRV.o: ../Source/USB_STACK/USB_DRV.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/USB_DRV.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/USB_DRV.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/USB_DRV.c  -o ${OBJECTDIR}/_ext/1669077864/USB_DRV.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/USB_DRV.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/USB_DRV.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o: ../Source/USB_STACK/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_function_cdc.c  -o ${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_function_cdc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o: ../Source/USB_STACK/usb_hal_pic24.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1669077864" 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o.d 
	@${RM} ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/USB_STACK/usb_hal_pic24.c  -o ${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1669077864/usb_hal_pic24.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/COMM_DAC.o: ../Source/COMM_DAC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/COMM_DAC.c  -o ${OBJECTDIR}/_ext/1728301206/COMM_DAC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/COMM_DAC.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/COMM_DAC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/COMM_SMC.o: ../Source/COMM_SMC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/COMM_SMC.c  -o ${OBJECTDIR}/_ext/1728301206/COMM_SMC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/COMM_SMC.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/COMM_SMC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o: ../Source/CPU_SM_MAIN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/CPU_SM_MAIN.c  -o ${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/CPU_SM_MAIN.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/CRC16.o: ../Source/CRC16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC16.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/CRC16.c  -o ${OBJECTDIR}/_ext/1728301206/CRC16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/CRC16.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/CRC16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/CRC32.o: ../Source/CRC32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/CRC32.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/CRC32.c  -o ${OBJECTDIR}/_ext/1728301206/CRC32.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/CRC32.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/CRC32.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_A2D.o: ../Source/DRV_A2D.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_A2D.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_A2D.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_A2D.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_A2D.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_I2C.o: ../Source/DRV_I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_I2C.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_I2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_I2C.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_I2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_MEM.o: ../Source/DRV_MEM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_MEM.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_MEM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_MEM.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_MEM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_RTC.o: ../Source/DRV_RTC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_RTC.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_RTC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_RTC.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_RTC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/EEPROM.o: ../Source/EEPROM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EEPROM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EEPROM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/EEPROM.c  -o ${OBJECTDIR}/_ext/1728301206/EEPROM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/EEPROM.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/EEPROM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/EVENTS.o: ../Source/EVENTS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EVENTS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/EVENTS.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/EVENTS.c  -o ${OBJECTDIR}/_ext/1728301206/EVENTS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/EVENTS.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/EVENTS.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/comm_host.o: ../Source/comm_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/comm_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/comm_host.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/comm_host.c  -o ${OBJECTDIR}/_ext/1728301206/comm_host.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/comm_host.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/comm_host.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/command_proc.o: ../Source/command_proc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/command_proc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/command_proc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/command_proc.c  -o ${OBJECTDIR}/_ext/1728301206/command_proc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/command_proc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/command_proc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o: ../Source/DRV_GLCD_SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/DRV_GLCD_SPI.c  -o ${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/DRV_GLCD_SPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1728301206/COMM_GSM.o: ../Source/COMM_GSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1728301206" 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Source/COMM_GSM.c  -o ${OBJECTDIR}/_ext/1728301206/COMM_GSM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1728301206/COMM_GSM.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1728301206/COMM_GSM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/Config_bit_setting.o: ../Config_bit_setting.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/Config_bit_setting.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Config_bit_setting.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Config_bit_setting.c  -o ${OBJECTDIR}/_ext/1472/Config_bit_setting.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/Config_bit_setting.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps -mlarge-code -mlarge-data -menable-large-arrays -O0 -I"../Header" -I"../Header/USB_STACK" -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Config_bit_setting.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    linker_script.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps --fill=0xFFDEAD  -mreserve=data@0x800:0x81B -mreserve=data@0x81C:0x81D -mreserve=data@0x81E:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x827 -mreserve=data@0x82A:0x84F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--no-handles,--no-isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--cref,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml,--no-ivt$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   linker_script.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  --save-temps --fill=0xFFDEAD -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--no-handles,--no-isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--cref,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml,--no-ivt$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/INSYS_SMCPU_USB.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
