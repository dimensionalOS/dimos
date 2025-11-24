import * as React from "npm:react"
import * as ReactDOMClient from "npm:react-dom/client"
import * as THREE from "npm:three"
import { Canvas, extend, Object3DNode, useThree } from "npm:@react-three/fiber"
import {
    Billboard,
    Line,
    OrbitControls,
    Plane,
    Text,
} from "npm:@react-three/drei"
import { Costmap, Drawable, Path, Vector } from "./types.ts"

// ───────────────────────────────────────────────────────────────────────────────
// Extend with OrbitControls
// ───────────────────────────────────────────────────────────────────────────────
extend({ OrbitControls })
declare global {
    namespace JSX {
        interface IntrinsicElements {
            orbitControls: Object3DNode<OrbitControls, typeof OrbitControls>
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────────
// Camera Controls
// ───────────────────────────────────────────────────────────────────────────────
function CameraControls() {
    const { camera, gl } = useThree()
    const controlsRef = React.useRef<OrbitControls>(null)

    React.useEffect(() => {
        if (controlsRef.current) {
            // Set initial camera position to better show the 3D effect
            camera.position.set(5, 8, 5)
            camera.lookAt(0, 0, 0)

            // Update controls with better settings for 3D viewing
            controlsRef.current.minDistance = 2
            controlsRef.current.maxDistance = 50
            controlsRef.current.update()
        }
    }, [camera])

    return (
        <OrbitControls
            ref={controlsRef}
            args={[camera, gl.domElement]}
            enableDamping
            dampingFactor={0.2}
            rotateSpeed={0.5}
            zoomSpeed={0.5}
        />
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Grid Component
// ───────────────────────────────────────────────────────────────────────────────
interface GridProps {
    size: number
    divisions: number
    color?: string
}

function Grid({ size = 10, divisions = 10, color = "#666666" }: GridProps) {
    return (
        <gridHelper
            args={[size, divisions, color, color]}
            position={[0, 0.01, 0]}
            rotation={[0, 0, 0]}
        />
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Costmap Component
// ───────────────────────────────────────────────────────────────────────────────
interface CostmapMeshProps {
    costmap: Costmap
}

function CostmapMesh({ costmap }: CostmapMeshProps) {
    const { grid, origin, resolution } = costmap
    const [rows, cols] = grid.shape

    // Calculate dimensions
    const width = cols * resolution
    const height = rows * resolution

    // Position is at the center of the grid
    const posX = origin.coords[0] + (width / 2)
    const posY = 0
    const posZ = origin.coords[1] + (height / 2)

    // Generate a 3D mesh directly from the costmap data
    const meshRef = React.useRef<THREE.Group>(null)

    // Create the mesh
    const colorData = React.useMemo(() => {
        const vertices: number[] = []
        const indices: number[] = []
        const colors: number[] = []

        // Cell size
        const cellWidth = width / cols
        const cellHeight = height / rows

        // Create vertices and colors
        for (let row = 0; row < rows; row++) {
            for (let col = 0; col < cols; col++) {
                const x = col * cellWidth - width / 2
                const z = row * cellHeight - height / 2

                // Get cost value
                const idx = row * cols + col
                const value = grid.data[idx]

                // Map cost value to height (y) with minimal elevation
                let y = 0
                if (value < 5) {
                    y = 0 // Flat for clear paths
                } else if (value < 20) {
                    y = (value / 20) * 0.1 // Barely visible bumps
                } else if (value > 80) {
                    y = 0.3 + ((value - 80) / 20) * 0.2 // Small obstacles
                } else {
                    y = 0.1 + ((value - 20) / 60) * 0.2 // Very gentle elevation
                }

                // Add the vertex
                vertices.push(x, y, z)

                // Determine color based on cost - original monochromatic scheme
                let r, g, b
                if (value < 5) {
                    // Very low cost - light gray (easily passable)
                    r = 0.9
                    g = 0.9
                    b = 0.9
                } else if (value < 20) {
                    // Low cost - slightly darker gray
                    const t = value / 20
                    const val = 0.9 - (t * 0.3)
                    r = val
                    g = val
                    b = val
                } else if (value > 80) {
                    // High cost - dark gray to black (obstacle)
                    const t = (value - 80) / 20
                    const val = 0.25 - (t * 0.25)
                    r = val
                    g = val
                    b = val
                } else {
                    // Medium cost - medium grays
                    const t = (value - 20) / 60
                    const val = 0.6 - (t * 0.35)
                    r = val
                    g = val
                    b = val
                }

                colors.push(r, g, b)
            }
        }

        // Create indices for triangles
        for (let row = 0; row < rows - 1; row++) {
            for (let col = 0; col < cols - 1; col++) {
                const a = row * cols + col
                const b = row * cols + col + 1
                const c = (row + 1) * cols + col
                const d = (row + 1) * cols + col + 1

                // First triangle
                indices.push(a, c, b)
                // Second triangle
                indices.push(b, c, d)
            }
        }

        return { vertices, indices, colors }
    }, [grid, rows, cols, width, height])

    return (
        <group position={[posX, 0, posZ]} ref={meshRef}>
            {/* Custom Mesh */}
            <mesh>
                <bufferGeometry>
                    <bufferAttribute
                        attach="index"
                        array={new Uint32Array(colorData.indices)}
                        count={colorData.indices.length}
                        itemSize={1}
                    />
                    <bufferAttribute
                        attach="attributes-position"
                        array={new Float32Array(colorData.vertices)}
                        count={colorData.vertices.length / 3}
                        itemSize={3}
                    />
                    <bufferAttribute
                        attach="attributes-color"
                        array={new Float32Array(colorData.colors)}
                        count={colorData.colors.length / 3}
                        itemSize={3}
                    />
                </bufferGeometry>
                <meshStandardMaterial
                    vertexColors
                    side={THREE.DoubleSide}
                />
            </mesh>

            {/* Optional wireframe overlay */}
            <mesh>
                <bufferGeometry>
                    <bufferAttribute
                        attach="index"
                        array={new Uint32Array(colorData.indices)}
                        count={colorData.indices.length}
                        itemSize={1}
                    />
                    <bufferAttribute
                        attach="attributes-position"
                        array={new Float32Array(colorData.vertices)}
                        count={colorData.vertices.length / 3}
                        itemSize={3}
                    />
                </bufferGeometry>
                <meshBasicMaterial
                    color="#ffffff"
                    wireframe
                    transparent
                    opacity={0.2}
                />
            </mesh>

            {/* Grid overlay - 10x coarser */}
            <Grid
                size={Math.max(width, height)}
                divisions={Math.max(
                    5,
                    Math.floor(Math.min(width, height) / (resolution * 10)),
                )}
            />
        </group>
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Path Component
// ───────────────────────────────────────────────────────────────────────────────
interface PathProps {
    path: Path
    color?: string
    label: string
}

function PathLine({ path, color = "#ff3333", label }: PathProps) {
    if (path.coords.length < 2) return null

    // Path height offset to float above terrain (33% lower)
    const pathHeight = 0.47

    // Convert 2D path coordinates to 3D points
    const points = path.coords.map(([x, y]) =>
        new THREE.Vector3(x, pathHeight, y)
    )

    // Calculate midpoint for label placement
    const midIdx = Math.floor(points.length / 2)
    const midPoint = points[midIdx]

    return (
        <group>
            {/* The path line */}
            <Line
                points={points}
                color={color}
                lineWidth={4}
            />

            {/* Path label - always faces camera */}
            <group position={[midPoint.x, pathHeight + 0.45, midPoint.z]}>
                <Billboard>
                    <Text
                        fontSize={0.2}
                        color="white"
                        anchorX="center"
                        anchorY="middle"
                        backgroundColor="#00000099"
                        padding={0.05}
                    >
                        {`${label} (${path.coords.length})`}
                    </Text>
                </Billboard>
            </group>

            {/* Start point marker */}
            <mesh position={[points[0].x, pathHeight + 0.1, points[0].z]}>
                <sphereGeometry args={[0.1, 16, 16]} />
                <meshBasicMaterial color="#ff3333" />
            </mesh>

            {/* End point marker */}
            <mesh
                position={[
                    points[points.length - 1].x,
                    pathHeight + 0.1,
                    points[points.length - 1].z,
                ]}
            >
                <sphereGeometry args={[0.1, 16, 16]} />
                <meshBasicMaterial color="#ff3333" />
            </mesh>

            {/* Vertical connectors to terrain - thicker */}
            {path.coords.map(([x, y], idx) => (
                <Line
                    key={`connector-${idx}`}
                    points={[
                        new THREE.Vector3(x, 0, y),
                        new THREE.Vector3(x, pathHeight, y),
                    ]}
                    color={color}
                    lineWidth={3}
                    opacity={0.5}
                    transparent
                />
            )).filter((_, idx) =>
                idx % 8 === 0 || idx === 0 || idx === path.coords.length - 1
            )} {/* Only show some connectors */}
        </group>
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Vector Component
// ───────────────────────────────────────────────────────────────────────────────
interface VectorProps {
    vector: Vector
    label: string
    color?: string
}

function VectorMarker({ vector, label, color = "#00aaff" }: VectorProps) {
    const [x, y] = vector.coords
    const markerHeight = 0.47 // Same height as paths (33% lower)

    return (
        <group>
            {/* Vector Marker - larger, no ring */}
            <mesh position={[x, markerHeight, y]}>
                <sphereGeometry args={[0.2, 16, 16]} />
                <meshBasicMaterial color={color} />
            </mesh>

            {/* Label - always faces camera */}
            <group position={[x, markerHeight + 0.5, y]}>
                <Billboard>
                    <Text
                        fontSize={0.2}
                        color="white"
                        anchorX="center"
                        anchorY="middle"
                        backgroundColor="#00000099"
                        padding={0.05}
                    >
                        {`${label} (${vector.coords[0].toFixed(2)}, ${
                            vector.coords[1].toFixed(2)
                        })`}
                    </Text>
                </Billboard>
            </group>

            {/* Vertical connector to terrain - thicker */}
            <Line
                points={[
                    new THREE.Vector3(x, 0, y),
                    new THREE.Vector3(x, markerHeight, y),
                ]}
                color={color}
                lineWidth={4}
                opacity={0.6}
                transparent
            />
        </group>
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Click Detector Component
// ───────────────────────────────────────────────────────────────────────────────
interface ClickDetectorProps {
    onWorldClick: (x: number, y: number) => void
}

function ClickDetector({ onWorldClick }: ClickDetectorProps) {
    const planeRef = React.useRef<THREE.Mesh>(null)

    const handleClick = (event: THREE.ThreeEvent<MouseEvent>) => {
        if (planeRef.current && event.intersections.length > 0) {
            // Get the intersection point with our invisible plane
            const intersection = event.intersections.find(
                (i) => i.object === planeRef.current,
            )

            if (intersection) {
                const point = intersection.point
                onWorldClick(point.x, point.z)
            }
        }
    }

    return (
        <mesh
            ref={planeRef}
            position={[0, 0, 0]}
            rotation={[-Math.PI / 2, 0, 0]}
            onClick={handleClick}
            visible={false}
        >
            <planeGeometry args={[100, 100]} />
            <meshBasicMaterial transparent opacity={0} />
        </mesh>
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Color Utilities
// ───────────────────────────────────────────────────────────────────────────────
// Tron-inspired color palette
const tronColors = [
    "#00FFFF", // Cyan
    "#00BFFF", // Deep Sky Blue
    "#1E90FF", // Dodger Blue
    "#40E0D0", // Turquoise
    "#00FF7F", // Spring Green
    "#7FFFD4", // Aquamarine
    "#48D1CC", // Medium Turquoise
    "#87CEFA", // Light Sky Blue
    "#0000FF", // Blue
    "#007FFF", // Azure
    "#4169E1", // Royal Blue
]

// Generate a consistent color based on the vector name
function getTronColor(name: string): string {
    // Hash the string to get a consistent index
    let hash = 0
    for (let i = 0; i < name.length; i++) {
        hash = ((hash << 5) - hash) + name.charCodeAt(i)
        hash |= 0 // Convert to 32bit integer
    }
    // Get positive index in the color array range
    const index = Math.abs(hash) % tronColors.length
    return tronColors[index]
}

// ───────────────────────────────────────────────────────────────────────────────
// Main Scene Component
// ───────────────────────────────────────────────────────────────────────────────
interface VisualizerSceneProps {
    state: Record<string, Drawable>
    onWorldClick: (x: number, y: number) => void
}

function VisualizerScene({ state, onWorldClick }: VisualizerSceneProps) {
    // Extract the costmaps, paths, and vectors from state
    const costmaps = Object.values(state).filter(
        (item): item is Costmap => item instanceof Costmap,
    )

    const pathEntries = Object.entries(state).filter(
        ([_, item]): item is Path => item instanceof Path,
    ) as [string, Path][]

    const vectorEntries = Object.entries(state).filter(
        ([_, item]): item is Vector => item instanceof Vector,
    ) as [string, Vector][]

    return (
        <Canvas
            shadows
            gl={{ antialias: true }}
            camera={{ position: [5, 8, 5], fov: 60 }}
            style={{ background: "#14151a" }}
        >
            {/* Ambient light for basic illumination */}
            <ambientLight intensity={0.4} />

            {/* Main directional light */}
            <directionalLight
                position={[10, 15, 10]}
                intensity={0.8}
                castShadow
            />

            {/* Additional lights for better 3D effect visibility */}
            <directionalLight
                position={[-10, 10, -10]}
                intensity={0.5}
                color="#8080ff"
            />

            {/* Add a point light to highlight elevation differences */}
            <pointLight
                position={[0, 10, 0]}
                intensity={0.7}
                distance={50}
                color="#ffffff"
            />

            {/* Camera controls */}
            <CameraControls />

            {/* Click detector */}
            <ClickDetector onWorldClick={onWorldClick} />

            {/* Render costmaps (bottom layer) */}
            {costmaps.map((costmap, index) => (
                <CostmapMesh key={`costmap-${index}`} costmap={costmap} />
            ))}

            {/* Render paths (middle layer) */}
            {pathEntries.map(([key, path]) => (
                <PathLine
                    key={`path-${key}`}
                    path={path}
                    label={key}
                    color="#ff3333"
                />
            ))}

            {/* Render vectors (top layer) */}
            {vectorEntries.map(([key, vector]) => (
                <VectorMarker
                    key={`vector-${key}`}
                    vector={vector}
                    label={key}
                    color={getTronColor(key)}
                />
            ))}
        </Canvas>
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// React Component
// ───────────────────────────────────────────────────────────────────────────────
const VisualizerComponent: React.FC<{
    state: Record<string, Drawable>
    onWorldClick: (x: number, y: number) => void
}> = ({
    state,
    onWorldClick,
}) => {
    return (
        <div
            className="visualizer-container"
            style={{ width: "100%", height: "100%" }}
        >
            <VisualizerScene state={state} onWorldClick={onWorldClick} />
        </div>
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Wrapper class (maintains API compatibility with previous visualizer)
// ───────────────────────────────────────────────────────────────────────────────
export class Visualizer {
    private container: HTMLElement | null
    private state: Record<string, Drawable> = {}
    private resizeObserver: ResizeObserver | null = null
    private root: ReactDOMClient.Root
    private onClickCallback: ((worldX: number, worldY: number) => void) | null =
        null

    constructor(selector: string) {
        this.container = document.querySelector(selector)
        if (!this.container) throw new Error(`Container not found: ${selector}`)
        this.root = ReactDOMClient.createRoot(this.container)

        // First paint
        this.render()

        // Keep canvas responsive
        if (window.ResizeObserver) {
            this.resizeObserver = new ResizeObserver(() => this.render())
            this.resizeObserver.observe(this.container)
        }
    }

    /** Register a callback for when user clicks on the visualization */
    public onWorldClick(
        callback: (worldX: number, worldY: number) => void,
    ): void {
        this.onClickCallback = callback
        this.render() // Re-render to apply the new callback
    }

    /** Handle click event from the 3D scene */
    private handleWorldClick = (x: number, y: number): void => {
        if (this.onClickCallback) {
            this.onClickCallback(x, y)
        }
    }

    /** Push a new application‑state snapshot to the visualiser */
    public visualizeState(state: Record<string, Drawable>): void {
        this.state = { ...state }
        this.render()
    }

    /** React‑render the component tree */
    private render(): void {
        this.root.render(
            <VisualizerComponent
                state={this.state}
                onWorldClick={this.handleWorldClick}
            />,
        )
    }

    /** Tear down listeners and free resources */
    public cleanup(): void {
        if (this.resizeObserver && this.container) {
            this.resizeObserver.unobserve(this.container)
            this.resizeObserver.disconnect()
        }
    }
}

// Convenience factory ----------------------------------------------------------
export function createReactVis(selector: string): Visualizer {
    return new Visualizer(selector)
}
