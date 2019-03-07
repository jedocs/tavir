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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../source/RF.c ../source/TimeDelay.c ../source/crc.c ../source/tavir.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/812168374/RF.o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ${OBJECTDIR}/_ext/812168374/crc.o ${OBJECTDIR}/_ext/812168374/tavir.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/812168374/RF.o.d ${OBJECTDIR}/_ext/812168374/TimeDelay.o.d ${OBJECTDIR}/_ext/812168374/crc.o.d ${OBJECTDIR}/_ext/812168374/tavir.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/812168374/RF.o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ${OBJECTDIR}/_ext/812168374/crc.o ${OBJECTDIR}/_ext/812168374/tavir.o

# Source Files
SOURCEFILES=../source/RF.c ../source/TimeDelay.c ../source/crc.c ../source/tavir.c


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24F16KA102
MP_LINKER_FILE_OPTION=,-Tp24F16KA102.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/812168374/RF.o: ../source/RF.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/RF.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/RF.o.ok ${OBJECTDIR}/_ext/812168374/RF.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/RF.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/RF.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/RF.o.d" -o ${OBJECTDIR}/_ext/812168374/RF.o ../source/RF.c    
	
${OBJECTDIR}/_ext/812168374/TimeDelay.o: ../source/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o.ok ${OBJECTDIR}/_ext/812168374/TimeDelay.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ../source/TimeDelay.c    
	
${OBJECTDIR}/_ext/812168374/crc.o: ../source/crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/crc.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/crc.o.ok ${OBJECTDIR}/_ext/812168374/crc.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/crc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/crc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/crc.o.d" -o ${OBJECTDIR}/_ext/812168374/crc.o ../source/crc.c    
	
${OBJECTDIR}/_ext/812168374/tavir.o: ../source/tavir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/tavir.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/tavir.o.ok ${OBJECTDIR}/_ext/812168374/tavir.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/tavir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/tavir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/tavir.o.d" -o ${OBJECTDIR}/_ext/812168374/tavir.o ../source/tavir.c    
	
else
${OBJECTDIR}/_ext/812168374/RF.o: ../source/RF.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/RF.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/RF.o.ok ${OBJECTDIR}/_ext/812168374/RF.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/RF.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/RF.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/RF.o.d" -o ${OBJECTDIR}/_ext/812168374/RF.o ../source/RF.c    
	
${OBJECTDIR}/_ext/812168374/TimeDelay.o: ../source/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o.ok ${OBJECTDIR}/_ext/812168374/TimeDelay.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ../source/TimeDelay.c    
	
${OBJECTDIR}/_ext/812168374/crc.o: ../source/crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/crc.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/crc.o.ok ${OBJECTDIR}/_ext/812168374/crc.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/crc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/crc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/crc.o.d" -o ${OBJECTDIR}/_ext/812168374/crc.o ../source/crc.c    
	
${OBJECTDIR}/_ext/812168374/tavir.o: ../source/tavir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/812168374 
	@${RM} ${OBJECTDIR}/_ext/812168374/tavir.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/tavir.o.ok ${OBJECTDIR}/_ext/812168374/tavir.o.err 
	@${RM} ${OBJECTDIR}/_ext/812168374/tavir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/tavir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/812168374/tavir.o.d" -o ${OBJECTDIR}/_ext/812168374/tavir.o ../source/tavir.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PICKIT2=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/tavir.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
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