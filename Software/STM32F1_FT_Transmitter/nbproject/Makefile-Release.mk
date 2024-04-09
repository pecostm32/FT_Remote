#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=arm-none-eabi-gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Release
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/5c0/crc16.o \
	${OBJECTDIR}/_ext/5c0/stm32f103_analog.o \
	${OBJECTDIR}/_ext/5c0/stm32f103_delay.o \
	${OBJECTDIR}/_ext/5c0/stm32f103_io.o \
	${OBJECTDIR}/_ext/5c0/stm32f103_nrf905.o \
	${OBJECTDIR}/stm32f103_ft_transmitter.o


# C Compiler Flags
CFLAGS=-Wall -Wno-write-strings -Wno-char-subscripts -fno-stack-protector -DNO_STDLIB=1 -O3 -mcpu=cortex-m3 -mthumb

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stm32f1_ft_transmitter

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stm32f1_ft_transmitter: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	arm-none-eabi-gcc -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stm32f1_ft_transmitter ${OBJECTFILES} ${LDLIBSOPTIONS} -T./stm32f103-64k.ld -nostdlib

${OBJECTDIR}/_ext/5c0/crc16.o: ../crc16.c
	${MKDIR} -p ${OBJECTDIR}/_ext/5c0
	${RM} "$@.d"
	$(COMPILE.c) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/5c0/crc16.o ../crc16.c

${OBJECTDIR}/_ext/5c0/stm32f103_analog.o: ../stm32f103_analog.c
	${MKDIR} -p ${OBJECTDIR}/_ext/5c0
	${RM} "$@.d"
	$(COMPILE.c) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/5c0/stm32f103_analog.o ../stm32f103_analog.c

${OBJECTDIR}/_ext/5c0/stm32f103_delay.o: ../stm32f103_delay.c
	${MKDIR} -p ${OBJECTDIR}/_ext/5c0
	${RM} "$@.d"
	$(COMPILE.c) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/5c0/stm32f103_delay.o ../stm32f103_delay.c

${OBJECTDIR}/_ext/5c0/stm32f103_io.o: ../stm32f103_io.c
	${MKDIR} -p ${OBJECTDIR}/_ext/5c0
	${RM} "$@.d"
	$(COMPILE.c) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/5c0/stm32f103_io.o ../stm32f103_io.c

${OBJECTDIR}/_ext/5c0/stm32f103_nrf905.o: ../stm32f103_nrf905.c
	${MKDIR} -p ${OBJECTDIR}/_ext/5c0
	${RM} "$@.d"
	$(COMPILE.c) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/5c0/stm32f103_nrf905.o ../stm32f103_nrf905.c

${OBJECTDIR}/stm32f103_ft_transmitter.o: stm32f103_ft_transmitter.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/stm32f103_ft_transmitter.o stm32f103_ft_transmitter.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
