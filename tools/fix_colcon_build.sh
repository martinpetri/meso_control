SRC="/workspaces/meso_control/install/meso_control_pkg/bin"
DEST="/workspaces/meso_control/install/meso_control_pkg/lib/meso_control_pkg"

rm -rf $DEST
mkdir $DEST
cp $SRC/* $DEST