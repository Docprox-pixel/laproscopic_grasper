import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime

# ==============================================================================
# SOURCES & REFERENCES:
# 1. sofa-framework/sofa: examples/Demos/liver.scn (FEM Logic)
# 2. ScheiklP/sofa_env: sofa_env/scenes/laparoscopy/common.py (Surgical Templates)
# 3. SofaDefrost/SofaViscoElastic: examples/TetrahedronViscoelasticityFEMForceField.py
# 4. SofaDefrost/SoftRobots: docs/tutorials/SoftGripper/details/step4.py (Collision)
# ==============================================================================

class SoftTissueModel:
    """
    Extracted and adapted soft tissue model for surgical simulation.
    Target: CPU Execution (AMD Ryzen Compatible).
    """
    def __init__(self, node, name="Liver", mass=1.0, young_modulus=5000, poisson_ratio=0.45):
        self.node = node.createChild(name)
        
        # 1. Physics Solvers (CPU Optimized)
        self.node.createObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        self.node.createObject('CGLinearSolver', name='linearSolver', iterations=25, tolerance=1e-9, threshold=1e-9)
        
        # 2. Topology & Mechanical State
        # Note: Using standard sofa mesh path. In production, provide absolute path to .msh/.vtk
        self.node.createObject('MeshGmshLoader', name='loader', filename='mesh/liver.msh')
        self.node.createObject('TetrahedronSetTopologyContainer', name='topology', src='@loader')
        self.node.createObject('MechanicalObject', name='dofs', template='Vec3d')
        
        # 3. Mass & Damping
        self.node.createObject('UniformMass', totalMass=mass)
        
        # 4. Force Field (FEM - Corotational for stable rotation)
        self.node.createObject('TetrahedralCorotationalFEMForceField', 
                               method='large', 
                               youngModulus=young_modulus, 
                               poissonRatio=poisson_ratio, 
                               computeGlobalMatrix=False)

        # 5. Fixed Constraints (Anchoring the organ)
        # Fixes a small region at the back to simulate attachment to ligaments
        self.node.createObject('FixedConstraint', indices='0 1 2 3 4 5')

        # 6. Collision Models (Multi-level contact)
        self.collision = self.node.createChild('Collision')
        self.collision.createObject('TriangleSetTopologyContainer', name='topology')
        self.collision.createObject('TriangleSetTopologyModifier')
        self.collision.createObject('Tetra2TriangleTopologicalMapping', inputName='@../topology', outputName='@topology')
        
        self.collision.createObject('MechanicalObject', name='collision_dofs')
        self.collision.createObject('IdentityMapping')
        
        # Triangle + Line + Point for robust interaction with surgical tools
        self.collision.createObject('TriangleCollisionModel', contactStiffness=10)
        self.collision.createObject('LineCollisionModel', contactStiffness=10)
        self.collision.createObject('PointCollisionModel', contactStiffness=10)

        # 7. Visual Representation
        self.visual = self.node.createChild('Visual')
        self.visual.createObject('OglModel', name='VisualModel', color=[0.8, 0.2, 0.2, 1.0])
        self.visual.createObject('IdentityMapping', input='@../dofs', output='@VisualModel')

def create_grasper_jaw(parent, name, transform=[0, 0, 0, 0, 0, 0, 1]):
    """Creates a rigid collision model for a grasper jaw."""
    jaw = parent.createChild(name)
    jaw.createObject('MechanicalObject', name='dofs', template='Rigid3d', position=transform)
    
    # Simple box visual/collision for the jaw
    collision = jaw.createChild('Collision')
    # Replace with specific STL loader if available
    collision.createObject('RegularGridTopology', name='grid', min=[-5, -1, -1], max=[5, 1, 1], nx=2, ny=2, nz=2)
    collision.createObject('MechanicalObject', name='dofs')
    collision.createObject('RigidMapping')
    
    collision.createObject('TriangleCollisionModel', moving=True, simulated=False)
    collision.createObject('LineCollisionModel', moving=True, simulated=False)
    collision.createObject('PointCollisionModel', moving=True, simulated=False)
    return jaw

def createScene(rootNode):
    # Required Plugins
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.Mass')
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.Visual')
    rootNode.createObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')
    rootNode.createObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')

    # Basic Scene Setup
    rootNode.gravity = [0, -9.81, 0]
    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-5)
    rootNode.createObject('DefaultPipeline')
    rootNode.createObject('BruteForceDetection')
    rootNode.createObject('DefaultContactManager', response='FrictionContactConstraint')
    rootNode.createObject('LocalMinDistance', name='Proximity', alarmDistance=0.5, contactDistance=0.1)

    # 1. Instantiate Soft Tissue (Liver)
    tissue = SoftTissueModel(rootNode, young_modulus=4500)

    # 2. Instantiate Rigid Grasper Jaws
    # Positioning them around the liver for interaction
    jaw1 = create_grasper_jaw(rootNode, "Jaw_Left",  transform=[10, 5, 0, 0, 0, 0, 1])
    jaw2 = create_grasper_jaw(rootNode, "Jaw_Right", transform=[10, -5, 0, 0, 0, 0, 1])

    return rootNode

if __name__ == "__main__":
    # Internal runner if executed directly
    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)
    print("Sofa Scene initialized successfully on CPU.")
