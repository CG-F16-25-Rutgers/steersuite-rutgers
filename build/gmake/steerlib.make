# GNU Make project makefile autogenerated by Premake
ifndef config
  config=debug
endif

ifndef verbose
  SILENT = @
endif

CC = clang
CXX = clang++
AR = ar

ifndef RESCOMP
  ifdef WINDRES
    RESCOMP = $(WINDRES)
  else
    RESCOMP = windres
  endif
endif

ifeq ($(config),debug)
  OBJDIR     = obj/Debug/steerlib
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libsteerlib.dylib
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DDEBUG
  INCLUDES  += -I../../steerlib/include -I../../external -I../../steerlib/include/util -I../../util/include
  ALL_CPPFLAGS  += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS    += $(CFLAGS) $(ALL_CPPFLAGS) $(ARCH) -Wall -Wextra -g -fPIC -stdlib=libc++ -std=c++0x -ggdb
  ALL_CXXFLAGS  += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS  += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  ALL_LDFLAGS   += $(LDFLAGS) -L. -L../lib -dynamiclib -stdlib=libc++ -Wl,-rpath,/Users/jason_ramirez/Desktop/fall-16-rutgers/computer-graphics/A3/steersuite-rutgers/build/lib -install_name @rpath/libsteerlib.dylib
  LDDEPS    += ../lib/libutil.dylib ../lib/libtinyxml.dylib
  LIBS      += $(LDDEPS) -framework OpenGL
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(ALL_LDFLAGS) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release)
  OBJDIR     = obj/Release/steerlib
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libsteerlib.dylib
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DNDEBUG
  INCLUDES  += -I../../steerlib/include -I../../external -I../../steerlib/include/util -I../../util/include
  ALL_CPPFLAGS  += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS    += $(CFLAGS) $(ALL_CPPFLAGS) $(ARCH) -Wall -Wextra -g -O2 -fPIC -stdlib=libc++ -std=c++0x -ggdb
  ALL_CXXFLAGS  += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS  += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  ALL_LDFLAGS   += $(LDFLAGS) -L. -L../lib -dynamiclib -stdlib=libc++ -Wl,-rpath,/Users/jason_ramirez/Desktop/fall-16-rutgers/computer-graphics/A3/steersuite-rutgers/build/lib -install_name @rpath/libsteerlib.dylib
  LDDEPS    += ../lib/libutil.dylib ../lib/libtinyxml.dylib
  LIBS      += $(LDDEPS) -framework OpenGL
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(ALL_LDFLAGS) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/AgentInterface.o \
	$(OBJDIR)/AgentMetricsCollector.o \
	$(OBJDIR)/AStarPlanner.o \
	$(OBJDIR)/BehaviorParameter.o \
	$(OBJDIR)/Behaviour.o \
	$(OBJDIR)/BenchmarkEngine.o \
	$(OBJDIR)/BoxObstacle.o \
	$(OBJDIR)/Camera.o \
	$(OBJDIR)/CircleObstacle.o \
	$(OBJDIR)/Clock.o \
	$(OBJDIR)/CommandLineParser.o \
	$(OBJDIR)/CompositeTechnique01.o \
	$(OBJDIR)/CompositeTechnique02.o \
	$(OBJDIR)/CompositeTechniqueEntropy.o \
	$(OBJDIR)/CompositeTechniqueGraph.o \
	$(OBJDIR)/CompositeTechniquePLE.o \
	$(OBJDIR)/Curve.o \
	$(OBJDIR)/DrawLib.o \
	$(OBJDIR)/DynamicLibrary.o \
	$(OBJDIR)/FrameSaver.o \
	$(OBJDIR)/GJK_EPA.o \
	$(OBJDIR)/GridDatabase2D.o \
	$(OBJDIR)/GridDatabasePlanningDomain.o \
	$(OBJDIR)/HighResCounter.o \
	$(OBJDIR)/MemoryMapper.o \
	$(OBJDIR)/Misc.o \
	$(OBJDIR)/OrientedBoxObstacle.o \
	$(OBJDIR)/OrientedWallObstacle.o \
	$(OBJDIR)/PerformanceProfiler.o \
	$(OBJDIR)/PolygonObstacle.o \
	$(OBJDIR)/RecFilePlayerModule.o \
	$(OBJDIR)/RecFileReader.o \
	$(OBJDIR)/RecFileWriter.o \
	$(OBJDIR)/SimulationEngine.o \
	$(OBJDIR)/SimulationMetricsCollector.o \
	$(OBJDIR)/SimulationOptions.o \
	$(OBJDIR)/SimulationRecorderModule.o \
	$(OBJDIR)/StateMachine.o \
	$(OBJDIR)/SteeringCommand.o \
	$(OBJDIR)/TestCasePlayerModule.o \
	$(OBJDIR)/TestCaseReader.o \
	$(OBJDIR)/TestCaseReaderPrivate.o \
	$(OBJDIR)/TestCaseWriter.o \
	$(OBJDIR)/ThreadedTaskManager.o \
	$(OBJDIR)/XMLParser.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking steerlib
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning steerlib
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
	$(SILENT) $(CXX) -x c++-header $(ALL_CXXFLAGS) -MMD -MP $(DEFINES) $(INCLUDES) -o "$@" -MF "$(@:%.gch=%.d)" -c "$<"
endif

$(OBJDIR)/AgentInterface.o: ../../steerlib/src/AgentInterface.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/AgentMetricsCollector.o: ../../steerlib/src/AgentMetricsCollector.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/AStarPlanner.o: ../../steerlib/src/AStarPlanner.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/BehaviorParameter.o: ../../steerlib/src/BehaviorParameter.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Behaviour.o: ../../steerlib/src/Behaviour.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/BenchmarkEngine.o: ../../steerlib/src/BenchmarkEngine.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/BoxObstacle.o: ../../steerlib/src/BoxObstacle.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Camera.o: ../../steerlib/src/Camera.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CircleObstacle.o: ../../steerlib/src/CircleObstacle.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Clock.o: ../../steerlib/src/Clock.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CommandLineParser.o: ../../steerlib/src/CommandLineParser.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CompositeTechnique01.o: ../../steerlib/src/CompositeTechnique01.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CompositeTechnique02.o: ../../steerlib/src/CompositeTechnique02.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CompositeTechniqueEntropy.o: ../../steerlib/src/CompositeTechniqueEntropy.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CompositeTechniqueGraph.o: ../../steerlib/src/CompositeTechniqueGraph.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CompositeTechniquePLE.o: ../../steerlib/src/CompositeTechniquePLE.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Curve.o: ../../steerlib/src/Curve.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/DrawLib.o: ../../steerlib/src/DrawLib.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/DynamicLibrary.o: ../../steerlib/src/DynamicLibrary.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/FrameSaver.o: ../../steerlib/src/FrameSaver.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/GJK_EPA.o: ../../steerlib/src/GJK_EPA.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/GridDatabase2D.o: ../../steerlib/src/GridDatabase2D.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/GridDatabasePlanningDomain.o: ../../steerlib/src/GridDatabasePlanningDomain.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/HighResCounter.o: ../../steerlib/src/HighResCounter.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/MemoryMapper.o: ../../steerlib/src/MemoryMapper.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Misc.o: ../../steerlib/src/Misc.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/OrientedBoxObstacle.o: ../../steerlib/src/OrientedBoxObstacle.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/OrientedWallObstacle.o: ../../steerlib/src/OrientedWallObstacle.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/PerformanceProfiler.o: ../../steerlib/src/PerformanceProfiler.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/PolygonObstacle.o: ../../steerlib/src/PolygonObstacle.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/RecFilePlayerModule.o: ../../steerlib/src/RecFilePlayerModule.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/RecFileReader.o: ../../steerlib/src/RecFileReader.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/RecFileWriter.o: ../../steerlib/src/RecFileWriter.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/SimulationEngine.o: ../../steerlib/src/SimulationEngine.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/SimulationMetricsCollector.o: ../../steerlib/src/SimulationMetricsCollector.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/SimulationOptions.o: ../../steerlib/src/SimulationOptions.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/SimulationRecorderModule.o: ../../steerlib/src/SimulationRecorderModule.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/StateMachine.o: ../../steerlib/src/StateMachine.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/SteeringCommand.o: ../../steerlib/src/SteeringCommand.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/TestCasePlayerModule.o: ../../steerlib/src/TestCasePlayerModule.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/TestCaseReader.o: ../../steerlib/src/TestCaseReader.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/TestCaseReaderPrivate.o: ../../steerlib/src/TestCaseReaderPrivate.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/TestCaseWriter.o: ../../steerlib/src/TestCaseWriter.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/ThreadedTaskManager.o: ../../steerlib/src/ThreadedTaskManager.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/XMLParser.o: ../../steerlib/src/XMLParser.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)
ifneq (,$(PCH))
  -include $(OBJDIR)/$(notdir $(PCH)).d
endif
