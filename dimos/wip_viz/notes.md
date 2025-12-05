Goals:
    - DONE: Get the foxglove vis working:
        - `python -m dimos.utils.cli.foxglove_bridge.run_foxglove_bridge`
    - DONE: See if there's a current rerun approach for iframe
    - DONE: Get the spy's working inside of zellij
    - DONE: Get the iframe working
    - DONE: Map rerun types to dimos / ros types
    - DONE-ish:Run dimos, have it update stuff in rerun
    - DONE: Get a listener for rerun
    - LATER: Decide how to get input out of the server/dashboard


Plan:
    - CREATED: Function that auto converts a dimos type to a rerun type
    - DONE: make a function that starts the DimOsDashboard
        - DONE: starts zellij
        - DONE: starts the webserver (rerun, zellij commands)
        - DONE: starts rerun sdk with super blueprint
    - DONE: Wrap it up in a module with inputs for rerun types
    - Function for sending that type to a rerun viewer
    - Modify the default module to scan its own outputs for reruns-compatible types

Cleanup later:
    - drag to resize terminals
    - PARTIAL: press enter automatically on the zellij windows
    - entity auto-target
    - use the "Unlock-First (non-colliding) preset" for zellij
    - escaping cmds in dashboard
    - `tccutil reset Camera` on macos
    - clean up the JS in the dashboard
    - clean up zellij sessions on ctrl+c
    - handle cmd+l, ctrl+l and just alt, etc better on zellij windows
        window.addEventListener("keydown", (e) => {
            if (e.key === "Tab" || e.key === "Escape" || e.key === " ") {
                e.preventDefault();   // stops browser default behavior
                e.stopPropagation();  // stops event before iframe receives it
            }
        }, true); // <-- use capture phase

Design:
    - modules inherit from AutoConnectiveModule
        - on init
            - scan outputs
            - if compatible with rerun, create add a `rerun` output
                - the rerun output probably needs:
                    - tuple of [(topic, rerun_type)]
    - create a rerun module
        - on init grabs the dimos_default_blueprint, starts the rerun server
            - keeps track of valid addresses based on the blueprint
        - generates an input for every supported dimos msg type based on the blueprint
        - generates visual events types as outputs (click, hover, waypoint, etc)
        - on input
            - rerun log the data, and volia it should get rendered, ignore or warn on stuff with incompatible address
    - TODO: how to handle the onclick event stuff
    - bonus:
        try to get this running: <iframe src="https://app.rerun.io/version/{RERUN_VERSION}/index.html?url={RRD_URL}"></iframe>
        zellij web in the same window 

Challeges:
    - Every rerun message has a target name, but those target names are blueprint-specific
        - We could have a blueprint module that offers endpoints, and then use the autoconnect system to connect
            - this is still kinda an issue for entities that have ID's
            - Can we dynamically add classes to the autoconnect system?