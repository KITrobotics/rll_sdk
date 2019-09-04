# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = python3 -msphinx
SOURCEDIR     = .
BUILDDIR      = _build

SSH_HOST=rll-doc.ipr.kit.edu
SSH_PORT=22
SSH_USER=rlldoc
SSH_TARGET_DIR=/var/www/rlldoc

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

upload: clean html
	# remove Modernizr until pull request is merged: https://github.com/readthedocs/sphinx_rtd_theme/pull/756
	sed -i '/<script type="text\/javascript" src="_static\/js\/modernizr.min.js"><\/script>/d' $(BUILDDIR)/html/*.html
	rsync -e "ssh -p $(SSH_PORT)" -P -rvzc --delete $(BUILDDIR)/ $(SSH_USER)@$(SSH_HOST):$(SSH_TARGET_DIR) --cvs-exclude

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
