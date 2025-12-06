Steps:
    - get RerunFunnel working
        - have Layout return a bunch of entity types and get passed to 
        - In of datatype
        - out of entity address
        - just calls rr.log (no other connections)
    - clean up dashboard
    - flag for auto-opening web browser
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