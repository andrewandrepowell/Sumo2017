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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=croutine.c event_groups.c list.c queue.c tasks.c timers.c port.c port_asm.S heap_2.c ConfigPerformance.c main.c UARTExt.c VCNL4010.c I2CExt.c MB85RC256V.c TCA9548A.c L293DNE.c DSP.c linear_transform_1.c grad_desc_4.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/croutine.o ${OBJECTDIR}/event_groups.o ${OBJECTDIR}/list.o ${OBJECTDIR}/queue.o ${OBJECTDIR}/tasks.o ${OBJECTDIR}/timers.o ${OBJECTDIR}/port.o ${OBJECTDIR}/port_asm.o ${OBJECTDIR}/heap_2.o ${OBJECTDIR}/ConfigPerformance.o ${OBJECTDIR}/main.o ${OBJECTDIR}/UARTExt.o ${OBJECTDIR}/VCNL4010.o ${OBJECTDIR}/I2CExt.o ${OBJECTDIR}/MB85RC256V.o ${OBJECTDIR}/TCA9548A.o ${OBJECTDIR}/L293DNE.o ${OBJECTDIR}/DSP.o ${OBJECTDIR}/linear_transform_1.o ${OBJECTDIR}/grad_desc_4.o
POSSIBLE_DEPFILES=${OBJECTDIR}/croutine.o.d ${OBJECTDIR}/event_groups.o.d ${OBJECTDIR}/list.o.d ${OBJECTDIR}/queue.o.d ${OBJECTDIR}/tasks.o.d ${OBJECTDIR}/timers.o.d ${OBJECTDIR}/port.o.d ${OBJECTDIR}/port_asm.o.d ${OBJECTDIR}/heap_2.o.d ${OBJECTDIR}/ConfigPerformance.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/UARTExt.o.d ${OBJECTDIR}/VCNL4010.o.d ${OBJECTDIR}/I2CExt.o.d ${OBJECTDIR}/MB85RC256V.o.d ${OBJECTDIR}/TCA9548A.o.d ${OBJECTDIR}/L293DNE.o.d ${OBJECTDIR}/DSP.o.d ${OBJECTDIR}/linear_transform_1.o.d ${OBJECTDIR}/grad_desc_4.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/croutine.o ${OBJECTDIR}/event_groups.o ${OBJECTDIR}/list.o ${OBJECTDIR}/queue.o ${OBJECTDIR}/tasks.o ${OBJECTDIR}/timers.o ${OBJECTDIR}/port.o ${OBJECTDIR}/port_asm.o ${OBJECTDIR}/heap_2.o ${OBJECTDIR}/ConfigPerformance.o ${OBJECTDIR}/main.o ${OBJECTDIR}/UARTExt.o ${OBJECTDIR}/VCNL4010.o ${OBJECTDIR}/I2CExt.o ${OBJECTDIR}/MB85RC256V.o ${OBJECTDIR}/TCA9548A.o ${OBJECTDIR}/L293DNE.o ${OBJECTDIR}/DSP.o ${OBJECTDIR}/linear_transform_1.o ${OBJECTDIR}/grad_desc_4.o

# Source Files
SOURCEFILES=croutine.c event_groups.c list.c queue.c tasks.c timers.c port.c port_asm.S heap_2.c ConfigPerformance.c main.c UARTExt.c VCNL4010.c I2CExt.c MB85RC256V.c TCA9548A.c L293DNE.c DSP.c linear_transform_1.c grad_desc_4.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX340F512H
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/port_asm.o: port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/port_asm.o.d 
	@${RM} ${OBJECTDIR}/port_asm.o 
	@${RM} ${OBJECTDIR}/port_asm.o.ok ${OBJECTDIR}/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/port_asm.o.d" "${OBJECTDIR}/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/port_asm.o.d"  -o ${OBJECTDIR}/port_asm.o port_asm.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1
	
else
${OBJECTDIR}/port_asm.o: port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/port_asm.o.d 
	@${RM} ${OBJECTDIR}/port_asm.o 
	@${RM} ${OBJECTDIR}/port_asm.o.ok ${OBJECTDIR}/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/port_asm.o.d" "${OBJECTDIR}/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/port_asm.o.d"  -o ${OBJECTDIR}/port_asm.o port_asm.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/port_asm.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/croutine.o: croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/croutine.o.d 
	@${RM} ${OBJECTDIR}/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/croutine.o.d" -o ${OBJECTDIR}/croutine.o croutine.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/event_groups.o: event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/event_groups.o.d 
	@${RM} ${OBJECTDIR}/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/event_groups.o.d" -o ${OBJECTDIR}/event_groups.o event_groups.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/list.o: list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/list.o.d 
	@${RM} ${OBJECTDIR}/list.o 
	@${FIXDEPS} "${OBJECTDIR}/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/list.o.d" -o ${OBJECTDIR}/list.o list.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/queue.o: queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/queue.o.d 
	@${RM} ${OBJECTDIR}/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/queue.o.d" -o ${OBJECTDIR}/queue.o queue.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/tasks.o: tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/tasks.o.d 
	@${RM} ${OBJECTDIR}/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/tasks.o.d" -o ${OBJECTDIR}/tasks.o tasks.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/timers.o: timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/timers.o.d 
	@${RM} ${OBJECTDIR}/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/timers.o.d" -o ${OBJECTDIR}/timers.o timers.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/port.o: port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/port.o.d 
	@${RM} ${OBJECTDIR}/port.o 
	@${FIXDEPS} "${OBJECTDIR}/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/port.o.d" -o ${OBJECTDIR}/port.o port.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/heap_2.o: heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/heap_2.o.d 
	@${RM} ${OBJECTDIR}/heap_2.o 
	@${FIXDEPS} "${OBJECTDIR}/heap_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/heap_2.o.d" -o ${OBJECTDIR}/heap_2.o heap_2.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/ConfigPerformance.o: ConfigPerformance.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ConfigPerformance.o.d 
	@${RM} ${OBJECTDIR}/ConfigPerformance.o 
	@${FIXDEPS} "${OBJECTDIR}/ConfigPerformance.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/ConfigPerformance.o.d" -o ${OBJECTDIR}/ConfigPerformance.o ConfigPerformance.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/UARTExt.o: UARTExt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UARTExt.o.d 
	@${RM} ${OBJECTDIR}/UARTExt.o 
	@${FIXDEPS} "${OBJECTDIR}/UARTExt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UARTExt.o.d" -o ${OBJECTDIR}/UARTExt.o UARTExt.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/VCNL4010.o: VCNL4010.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/VCNL4010.o.d 
	@${RM} ${OBJECTDIR}/VCNL4010.o 
	@${FIXDEPS} "${OBJECTDIR}/VCNL4010.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/VCNL4010.o.d" -o ${OBJECTDIR}/VCNL4010.o VCNL4010.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/I2CExt.o: I2CExt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2CExt.o.d 
	@${RM} ${OBJECTDIR}/I2CExt.o 
	@${FIXDEPS} "${OBJECTDIR}/I2CExt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2CExt.o.d" -o ${OBJECTDIR}/I2CExt.o I2CExt.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/MB85RC256V.o: MB85RC256V.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MB85RC256V.o.d 
	@${RM} ${OBJECTDIR}/MB85RC256V.o 
	@${FIXDEPS} "${OBJECTDIR}/MB85RC256V.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MB85RC256V.o.d" -o ${OBJECTDIR}/MB85RC256V.o MB85RC256V.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/TCA9548A.o: TCA9548A.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/TCA9548A.o.d 
	@${RM} ${OBJECTDIR}/TCA9548A.o 
	@${FIXDEPS} "${OBJECTDIR}/TCA9548A.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/TCA9548A.o.d" -o ${OBJECTDIR}/TCA9548A.o TCA9548A.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/L293DNE.o: L293DNE.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/L293DNE.o.d 
	@${RM} ${OBJECTDIR}/L293DNE.o 
	@${FIXDEPS} "${OBJECTDIR}/L293DNE.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/L293DNE.o.d" -o ${OBJECTDIR}/L293DNE.o L293DNE.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/DSP.o: DSP.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/DSP.o.d 
	@${RM} ${OBJECTDIR}/DSP.o 
	@${FIXDEPS} "${OBJECTDIR}/DSP.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DSP.o.d" -o ${OBJECTDIR}/DSP.o DSP.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/linear_transform_1.o: linear_transform_1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/linear_transform_1.o.d 
	@${RM} ${OBJECTDIR}/linear_transform_1.o 
	@${FIXDEPS} "${OBJECTDIR}/linear_transform_1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/linear_transform_1.o.d" -o ${OBJECTDIR}/linear_transform_1.o linear_transform_1.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/grad_desc_4.o: grad_desc_4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/grad_desc_4.o.d 
	@${RM} ${OBJECTDIR}/grad_desc_4.o 
	@${FIXDEPS} "${OBJECTDIR}/grad_desc_4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/grad_desc_4.o.d" -o ${OBJECTDIR}/grad_desc_4.o grad_desc_4.c      $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/croutine.o: croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/croutine.o.d 
	@${RM} ${OBJECTDIR}/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/croutine.o.d" -o ${OBJECTDIR}/croutine.o croutine.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/event_groups.o: event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/event_groups.o.d 
	@${RM} ${OBJECTDIR}/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/event_groups.o.d" -o ${OBJECTDIR}/event_groups.o event_groups.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/list.o: list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/list.o.d 
	@${RM} ${OBJECTDIR}/list.o 
	@${FIXDEPS} "${OBJECTDIR}/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/list.o.d" -o ${OBJECTDIR}/list.o list.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/queue.o: queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/queue.o.d 
	@${RM} ${OBJECTDIR}/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/queue.o.d" -o ${OBJECTDIR}/queue.o queue.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/tasks.o: tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/tasks.o.d 
	@${RM} ${OBJECTDIR}/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/tasks.o.d" -o ${OBJECTDIR}/tasks.o tasks.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/timers.o: timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/timers.o.d 
	@${RM} ${OBJECTDIR}/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/timers.o.d" -o ${OBJECTDIR}/timers.o timers.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/port.o: port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/port.o.d 
	@${RM} ${OBJECTDIR}/port.o 
	@${FIXDEPS} "${OBJECTDIR}/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/port.o.d" -o ${OBJECTDIR}/port.o port.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/heap_2.o: heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/heap_2.o.d 
	@${RM} ${OBJECTDIR}/heap_2.o 
	@${FIXDEPS} "${OBJECTDIR}/heap_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/heap_2.o.d" -o ${OBJECTDIR}/heap_2.o heap_2.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/ConfigPerformance.o: ConfigPerformance.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ConfigPerformance.o.d 
	@${RM} ${OBJECTDIR}/ConfigPerformance.o 
	@${FIXDEPS} "${OBJECTDIR}/ConfigPerformance.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/ConfigPerformance.o.d" -o ${OBJECTDIR}/ConfigPerformance.o ConfigPerformance.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/UARTExt.o: UARTExt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UARTExt.o.d 
	@${RM} ${OBJECTDIR}/UARTExt.o 
	@${FIXDEPS} "${OBJECTDIR}/UARTExt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UARTExt.o.d" -o ${OBJECTDIR}/UARTExt.o UARTExt.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/VCNL4010.o: VCNL4010.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/VCNL4010.o.d 
	@${RM} ${OBJECTDIR}/VCNL4010.o 
	@${FIXDEPS} "${OBJECTDIR}/VCNL4010.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/VCNL4010.o.d" -o ${OBJECTDIR}/VCNL4010.o VCNL4010.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/I2CExt.o: I2CExt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2CExt.o.d 
	@${RM} ${OBJECTDIR}/I2CExt.o 
	@${FIXDEPS} "${OBJECTDIR}/I2CExt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2CExt.o.d" -o ${OBJECTDIR}/I2CExt.o I2CExt.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/MB85RC256V.o: MB85RC256V.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MB85RC256V.o.d 
	@${RM} ${OBJECTDIR}/MB85RC256V.o 
	@${FIXDEPS} "${OBJECTDIR}/MB85RC256V.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MB85RC256V.o.d" -o ${OBJECTDIR}/MB85RC256V.o MB85RC256V.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/TCA9548A.o: TCA9548A.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/TCA9548A.o.d 
	@${RM} ${OBJECTDIR}/TCA9548A.o 
	@${FIXDEPS} "${OBJECTDIR}/TCA9548A.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/TCA9548A.o.d" -o ${OBJECTDIR}/TCA9548A.o TCA9548A.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/L293DNE.o: L293DNE.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/L293DNE.o.d 
	@${RM} ${OBJECTDIR}/L293DNE.o 
	@${FIXDEPS} "${OBJECTDIR}/L293DNE.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/L293DNE.o.d" -o ${OBJECTDIR}/L293DNE.o L293DNE.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/DSP.o: DSP.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/DSP.o.d 
	@${RM} ${OBJECTDIR}/DSP.o 
	@${FIXDEPS} "${OBJECTDIR}/DSP.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DSP.o.d" -o ${OBJECTDIR}/DSP.o DSP.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/linear_transform_1.o: linear_transform_1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/linear_transform_1.o.d 
	@${RM} ${OBJECTDIR}/linear_transform_1.o 
	@${FIXDEPS} "${OBJECTDIR}/linear_transform_1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/linear_transform_1.o.d" -o ${OBJECTDIR}/linear_transform_1.o linear_transform_1.c      $(COMPARISON_BUILD) 
	
${OBJECTDIR}/grad_desc_4.o: grad_desc_4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/grad_desc_4.o.d 
	@${RM} ${OBJECTDIR}/grad_desc_4.o 
	@${FIXDEPS} "${OBJECTDIR}/grad_desc_4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/grad_desc_4.o.d" -o ${OBJECTDIR}/grad_desc_4.o grad_desc_4.c      $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}            $(COMPARISON_BUILD)    -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=_min_heap_size=2048,--defsym=_min_stack_size=2048,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}            $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=2048,--defsym=_min_stack_size=2048,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Sumo2017.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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
