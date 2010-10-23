.SUFFIXES: .erl .hrl

ESRC=.
EBIN=../ebin
ERLC=erlc

MODULES= \
	eSBGE \
	brownMesh \
	camera \
	graphicsScene \
	instancedModel \
	light

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
