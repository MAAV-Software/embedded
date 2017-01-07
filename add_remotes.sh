#!/bin/bash

# All uniqnames with remotes for ctrl
# A L P H A B E T I C A L !
REMOTES="
	ashnich\
	clweng\
	dichencd\
	evansco\
	lozinski\
	neckardt\
	pedixon\
	psasawat\
	shihapan\
	sajanptl\
	sabbaghp\
	spschul\
	tayade\
	tjha\
	topipari\
	zmpan\
	zzwang\
"

CURRREMOTES=$(git remote)

for remote in ${REMOTES}
do
if [ -z `git remote | grep ${remote}` ] 
then
	echo "Adding remote for ${remote}"
	git remote add ${remote} git@git.maavumich.org:ctrl-${remote}.git
fi
done
