# command-center-extension

This is a Foxglove extension for visualizing robot data and controlling the robot. See `dimos/web/websocket_vis/README.md` for how to use the module in your robot.

## Build and use

Install the Foxglove Studio desktop application.

Install the Node dependencies:

    npm install

Build the package and install it into Foxglove:

    npm run build && npm run local-install

To add the panel, go to Foxglove Studio, click on the "Add panel" icon on the top right and select "command-center [local]".
