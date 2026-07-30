# Trust the parametrizer to preserve collision validity

DimOS does not independently collision-check every sample of a returned timed trajectory during plan materialization. A parametrization backend that fits a curve away from the source waypoint polyline is responsible for collision-checking that curve against its authoritative world; DimOS validates the returned trajectory's structure and motion limits without duplicating the backend's potentially expensive collision pass.
