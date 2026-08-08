# Keep box-filling behavior out of the generic pick/place interface

The generic pick/place interface contains object selection and single-object physical transactions, not open-box measurement, fit policy, rim-clearance calculations, or repeated collection behavior. A derived box-filling module adds those application-specific capabilities; its blueprint only composes and configures that module and does not define or gate the module's interface. Generic manipulation callers therefore never need to learn the box-filling surface.
