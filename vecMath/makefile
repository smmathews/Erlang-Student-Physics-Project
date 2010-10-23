# makefile for PHY350 Spring 2008 by Shane M Mathews
# All content (C) 2008 DigiPen (USA) Corporation, all rights reserved.
.SUFFIXES: .erl .hrl

ESRC=.
EBIN=../ebin
ERLC=erlc

MODULES= \
	quaternion \
	vec3 \
	mat3

TARGET_FILES= $(MODULES:%=$(EBIN)/%.beam)

# ----------------------------------------------------
# FLAGS
# ----------------------------------------------------
ERL_FLAGS += 
ERL_COMPILE_FLAGS += -W $(TYPE_FLAGS)

# ----------------------------------------------------
# Targets
# ----------------------------------------------------

common: $(TARGET_FILES) 

clean:
	rm -f $(TARGET_FILES)

$(EBIN)/%.beam: $(ESRC)/%.erl
	$(ERLC) $(ERL_FLAGS) $(ERL_COMPILE_FLAGS) -o$(EBIN) $<