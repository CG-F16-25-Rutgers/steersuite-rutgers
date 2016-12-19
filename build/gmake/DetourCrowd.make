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
  OBJDIR     = obj/Debug/DetourCrowd
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libDetourCrowd.a
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DDEBUG
  INCLUDES  += -I../../navmeshBuilder/include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../external/recastnavigation/DetourCrowd/Include -I../../util/include
  ALL_CPPFLAGS  += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS    += $(CFLAGS) $(ALL_CPPFLAGS) $(ARCH) -Wall -Wextra -g -stdlib=libc++ -std=c++0x -ggdb -fPIC
  ALL_CXXFLAGS  += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS  += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  ALL_LDFLAGS   += $(LDFLAGS) -L. -L../lib -stdlib=libc++ -Wl,-rpath,/Users/jason_ramirez/Desktop/fall-16-rutgers/computer-graphics/A3/steersuite-rutgers/build/lib -install_name @rpath/libDetourCrowd.dylib
  LDDEPS    += ../lib/libRecast.a ../lib/libDetour.a
  LIBS      += $(LDDEPS) -framework OpenGL
  LINKCMD    = $(AR) -rcs $(TARGET) $(OBJECTS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release)
  OBJDIR     = obj/Release/DetourCrowd
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libDetourCrowd.a
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DNDEBUG
  INCLUDES  += -I../../navmeshBuilder/include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../external/recastnavigation/DetourCrowd/Include -I../../util/include
  ALL_CPPFLAGS  += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS    += $(CFLAGS) $(ALL_CPPFLAGS) $(ARCH) -Wall -Wextra -g -O2 -stdlib=libc++ -std=c++0x -ggdb -fPIC
  ALL_CXXFLAGS  += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS  += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  ALL_LDFLAGS   += $(LDFLAGS) -L. -L../lib -stdlib=libc++ -Wl,-rpath,/Users/jason_ramirez/Desktop/fall-16-rutgers/computer-graphics/A3/steersuite-rutgers/build/lib -install_name @rpath/libDetourCrowd.dylib
  LDDEPS    += ../lib/libRecast.a ../lib/libDetour.a
  LIBS      += $(LDDEPS) -framework OpenGL
  LINKCMD    = $(AR) -rcs $(TARGET) $(OBJECTS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/DetourCrowd.o \
	$(OBJDIR)/DetourLocalBoundary.o \
	$(OBJDIR)/DetourObstacleAvoidance.o \
	$(OBJDIR)/DetourPathCorridor.o \
	$(OBJDIR)/DetourPathQueue.o \
	$(OBJDIR)/DetourProximityGrid.o \

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
	@echo Linking DetourCrowd
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
	@echo Cleaning DetourCrowd
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

$(OBJDIR)/DetourCrowd.o: ../../external/recastnavigation/DetourCrowd/Source/DetourCrowd.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/DetourLocalBoundary.o: ../../external/recastnavigation/DetourCrowd/Source/DetourLocalBoundary.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/DetourObstacleAvoidance.o: ../../external/recastnavigation/DetourCrowd/Source/DetourObstacleAvoidance.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/DetourPathCorridor.o: ../../external/recastnavigation/DetourCrowd/Source/DetourPathCorridor.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/DetourPathQueue.o: ../../external/recastnavigation/DetourCrowd/Source/DetourPathQueue.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/DetourProximityGrid.o: ../../external/recastnavigation/DetourCrowd/Source/DetourProximityGrid.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)
ifneq (,$(PCH))
  -include $(OBJDIR)/$(notdir $(PCH)).d
endif