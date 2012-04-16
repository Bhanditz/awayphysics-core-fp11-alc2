package awayphysics.dynamics {
	import C_Run.addBodyInC;
	import C_Run.addBodyWithGroupInC;
	import C_Run.addCharacterInC;
	import C_Run.addConstraintInC;
	import C_Run.addVehicleInC;
	import C_Run.createDiscreteDynamicsWorldWithAxisSweep3InC;
	import C_Run.createDiscreteDynamicsWorldWithDbvtInC;
	import C_Run.initLib;
	import C_Run.physicsStepInC;
	import C_Run.removeBodyInC;
	import C_Run.removeCharacterInC;
	import C_Run.removeConstraintInC;
	import C_Run.removeVehicleInC;
	import C_Run.removeCollisionObjectInC;
	
	import awayphysics.AWPBase;
	import awayphysics.collision.dispatch.AWPCollisionObject;
	import awayphysics.collision.dispatch.AWPCollisionWorld;
	import awayphysics.collision.shapes.AWPBvhTriangleMeshShape;
	import awayphysics.collision.shapes.AWPCompoundShape;
	import awayphysics.collision.shapes.AWPConvexHullShape;
	import awayphysics.collision.shapes.AWPHeightfieldTerrainShape;
	import awayphysics.data.AWPCollisionFlags;
	import awayphysics.data.AWPCollisionShapeType;
	import awayphysics.dynamics.character.AWPKinematicCharacterController;
	import awayphysics.dynamics.constraintsolver.AWPTypedConstraint;
	import awayphysics.dynamics.vehicle.AWPRaycastVehicle;
	import awayphysics.math.AWPVector3;
	
	import com.adobe.alchemy.AlcConsole;
	import com.adobe.alchemy.CModule;
	
	import flash.geom.Vector3D;
	import flash.utils.Dictionary;

	public class AWPDynamicsWorld extends AWPCollisionWorld {
		private static var currentDynamicsWorld : AWPDynamicsWorld;
		private var m_gravity : AWPVector3;
		private var m_rigidBodies : Vector.<AWPRigidBody>;
		private var m_nonStaticRigidBodies : Vector.<AWPRigidBody>;
		private var m_vehicles : Vector.<AWPRaycastVehicle>;
		private var m_characters : Vector.<AWPKinematicCharacterController>;
		private var m_constraints:Vector.<AWPTypedConstraint>;

		public static function getInstance() : AWPDynamicsWorld {
			if (!currentDynamicsWorld) {
				trace("version: AwayPhysics v1.0 alpha (14-4-2012)");
				currentDynamicsWorld = new AWPDynamicsWorld();
			}
			return currentDynamicsWorld;
		}

		public function AWPDynamicsWorld() {
			initLib(this);
			new AlcConsole(this);
			m_rigidBodies = new Vector.<AWPRigidBody>();
			m_nonStaticRigidBodies = new Vector.<AWPRigidBody>();
			m_vehicles = new Vector.<AWPRaycastVehicle>();
			m_characters = new Vector.<AWPKinematicCharacterController>();
			m_constraints = new Vector.<AWPTypedConstraint>();
		}

		/**
		 * init the physics world with btDbvtBroadphase
		 * refer to http://bulletphysics.org/mediawiki-1.5.8/index.php/Broadphase
		 */
		public function initWithDbvtBroadphase() : void {
			pointer = createDiscreteDynamicsWorldWithDbvtInC();
			m_gravity = new AWPVector3(pointer + 224);
			this.gravity = new Vector3D(0, -10, 0);
		}

		/**
		 * init the physics world with btAxisSweep3
		 * refer to http://bulletphysics.org/mediawiki-1.5.8/index.php/Broadphase
		 */
		public function initWithAxisSweep3(worldAabbMin : Vector3D, worldAabbMax : Vector3D) : void {
			var vec1:AWPVector3 = new AWPVector3();
			vec1.sv3d = worldAabbMin;
			var vec2:AWPVector3 = new AWPVector3();
			vec2.sv3d = worldAabbMax;
			pointer = createDiscreteDynamicsWorldWithAxisSweep3InC(vec1.pointer, vec2.pointer);
			CModule.free(vec1.pointer);
			CModule.free(vec2.pointer);
			m_gravity = new AWPVector3(pointer + 224);
			this.gravity = new Vector3D(0, -10, 0);
		}

		/**
		 * add a rigidbody to physics world
		 */
		public function addRigidBody(body : AWPRigidBody) : void {
			addBodyInC(body.pointer);

			if (body.collisionFlags != AWPCollisionFlags.CF_STATIC_OBJECT) {
				if (m_nonStaticRigidBodies.indexOf(body) < 0) {
					m_nonStaticRigidBodies.push(body);
				}
			}
			if (m_rigidBodies.indexOf(body) < 0) {
				m_rigidBodies.push(body);
			}
			if(!m_collisionObjects.hasOwnProperty(body.pointer.toString())){
				m_collisionObjects[body.pointer.toString()] = body;
			}
		}

		/**
		 * add a rigidbody to physics world with group and mask
		 * refer to: http://bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
		 */
		public function addRigidBodyWithGroup(body : AWPRigidBody, group : int, mask : int) : void {
			addBodyWithGroupInC(body.pointer, group, mask);

			if (body.collisionFlags != AWPCollisionFlags.CF_STATIC_OBJECT) {
				if (m_nonStaticRigidBodies.indexOf(body) < 0) {
					m_nonStaticRigidBodies.push(body);
				}
			}
			if (m_rigidBodies.indexOf(body) < 0) {
				m_rigidBodies.push(body);
			}
			if(!m_collisionObjects.hasOwnProperty(body.pointer.toString())){
				m_collisionObjects[body.pointer.toString()] = body;
			}
		}

		/**
		 * remove a rigidbody from physics world
		 */
		public function removeRigidBody(body : AWPRigidBody) : void {
			body.removeAllRays();
			if(body.shape.shapeType==AWPCollisionShapeType.TRIANGLE_MESH_SHAPE){
				AWPBvhTriangleMeshShape(body.shape).deleteBvhTriangleMeshShapeBuffer();
			}else if(body.shape.shapeType==AWPCollisionShapeType.CONVEX_HULL_SHAPE){
				AWPConvexHullShape(body.shape).deleteConvexHullShapeBuffer();
			}else if(body.shape.shapeType==AWPCollisionShapeType.HEIGHT_FIELD_TERRAIN){
				AWPHeightfieldTerrainShape(body.shape).deleteHeightfieldTerrainShapeBuffer();
			}else if(body.shape.shapeType==AWPCollisionShapeType.COMPOUND_SHAPE){
				AWPCompoundShape(body.shape).removeAllChildren();
			}
			removeBodyInC(body.pointer);

			if (m_nonStaticRigidBodies.indexOf(body) >= 0) {
				m_nonStaticRigidBodies.splice(m_nonStaticRigidBodies.indexOf(body), 1);
			}
			if (m_rigidBodies.indexOf(body) >= 0) {
				m_rigidBodies.splice(m_rigidBodies.indexOf(body), 1);
			}
			if(m_collisionObjects.hasOwnProperty(body.pointer.toString())){
				delete m_collisionObjects[body.pointer.toString()];
			}
		}
		
		/**
		 * add a constraint to physics world
		 */
		public function addConstraint(constraint : AWPTypedConstraint, disableCollisionsBetweenLinkedBodies : Boolean = false) : void {
			addConstraintInC(constraint.pointer, disableCollisionsBetweenLinkedBodies ? 1 : 0);
			
			if (m_constraints.indexOf(constraint) < 0) {
				m_constraints.push(constraint);
			}
		}
		
		/**
		 * remove a constraint from physics world
		 */
		public function removeConstraint(constraint : AWPTypedConstraint) : void {
			removeConstraintInC(constraint.pointer);
			
			if (m_constraints.indexOf(constraint) >= 0) {
				m_constraints.splice(m_constraints.indexOf(constraint), 1);
			}
		}
		
		/**
		 * add a vehicle to physics world
		 */
		public function addVehicle(vehicle : AWPRaycastVehicle) : void {
			addVehicleInC(vehicle.pointer);

			if (m_vehicles.indexOf(vehicle) < 0) {
				m_vehicles.push(vehicle);
			}
		}
		
		/**
		 * remove a vehicle from physics world
		 */
		public function removeVehicle(vehicle : AWPRaycastVehicle) : void {
			removeRigidBody(vehicle.getRigidBody());
			removeVehicleInC(vehicle.pointer);

			if (m_vehicles.indexOf(vehicle) >= 0) {
				m_vehicles.splice(m_vehicles.indexOf(vehicle), 1);
			}
		}
		
		/**
		 * add a character to physics world
		 */
		public function addCharacter(character : AWPKinematicCharacterController, group : int = 32, mask : int = -1) : void {
			addCharacterInC(character.pointer, group, mask);

			if (m_characters.indexOf(character) < 0) {
				m_characters.push(character);
			}
			
			if(!m_collisionObjects.hasOwnProperty(character.ghostObject.pointer.toString())){
				m_collisionObjects[character.ghostObject.pointer.toString()] = character.ghostObject;
			}
		}
		
		/**
		 * remove a character from physics world
		 */
		public function removeCharacter(character : AWPKinematicCharacterController) : void {
			character.ghostObject.removeAllRays();
			if(character.shape.shapeType==AWPCollisionShapeType.CONVEX_HULL_SHAPE){
				AWPConvexHullShape(character.shape).deleteConvexHullShapeBuffer();
			}else if(character.shape.shapeType==AWPCollisionShapeType.COMPOUND_SHAPE){
				AWPCompoundShape(character.shape).removeAllChildren();
			}
			removeCharacterInC(character.pointer);

			if (m_characters.indexOf(character) >= 0) {
				m_characters.splice(m_characters.indexOf(character), 1);
			}
			if(m_collisionObjects.hasOwnProperty(character.ghostObject.pointer.toString())){
				delete m_collisionObjects[character.ghostObject.pointer.toString()];
			}
		}
		
		/**
		 * clear all objects from physics world
		 */
		public function cleanWorld():void{
			while (m_constraints.length > 0){
				removeConstraint(m_constraints[0]);
			}
			m_constraints.length = 0;
			
			while (m_vehicles.length > 0){
				removeVehicle(m_vehicles[0]);
			}
			m_vehicles.length = 0;
			
			while (m_characters.length > 0){
				removeCharacter(m_characters[0]);
			}
			m_characters.length = 0;
			
			while (m_rigidBodies.length > 0){
				removeRigidBody(m_rigidBodies[0]);
			}
			m_nonStaticRigidBodies.length = 0;
			m_rigidBodies.length = 0;
			
			for each (var obj:AWPCollisionObject in m_collisionObjects) {
				obj.removeAllRays();
				if(obj.shape.shapeType==AWPCollisionShapeType.TRIANGLE_MESH_SHAPE){
					AWPBvhTriangleMeshShape(obj.shape).deleteBvhTriangleMeshShapeBuffer();
				}else if(obj.shape.shapeType==AWPCollisionShapeType.CONVEX_HULL_SHAPE){
					AWPConvexHullShape(obj.shape).deleteConvexHullShapeBuffer();
				}else if(obj.shape.shapeType==AWPCollisionShapeType.HEIGHT_FIELD_TERRAIN){
					AWPHeightfieldTerrainShape(obj.shape).deleteHeightfieldTerrainShapeBuffer();
				}else if(obj.shape.shapeType==AWPCollisionShapeType.COMPOUND_SHAPE){
					AWPCompoundShape(obj.shape).removeAllChildren();
				}
				removeCollisionObjectInC(obj.pointer);
			}
			m_collisionObjects =  new Dictionary(true);
		}

		/**
		 * get the gravity of physics world
		 */
		public function get gravity() : Vector3D {
			return m_gravity.v3d;
		}

		/**
		 * set the gravity of physics world
		 */
		public function set gravity(g : Vector3D) : void {
			m_gravity.v3d = g;
			for each (var body:AWPRigidBody in m_nonStaticRigidBodies) {
				body.gravity = g;
			}
		}

		/**
		 * get all rigidbodies
		 */
		public function get rigidBodies() : Vector.<AWPRigidBody> {
			return m_rigidBodies;
		}

		/**
		 * get all non static rigidbodies
		 */
		public function get nonStaticRigidBodies() : Vector.<AWPRigidBody> {
			return m_nonStaticRigidBodies;
		}
		
		public function get constraints() : Vector.<AWPTypedConstraint> {
			return m_constraints;
		}

		public function get vehicles() : Vector.<AWPRaycastVehicle> {
			return m_vehicles;
		}

		public function get characters() : Vector.<AWPKinematicCharacterController> {
			return m_characters;
		}

		/**
		 * set physics world scaling
		 * refer to http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=Scaling_The_World
		 */
		public function set scaling(v : Number) : void {
			_scaling = v;
		}

		/**
		 * get physics world scaling
		 */
		public function get scaling() : Number {
			return _scaling;
		}

		/**
		 * get if implement object collision callback
		 */
		public function get collisionCallbackOn() : Boolean {
			return CModule.read8(pointer + 247) == 1;
		}

		/**
		 * set this to true if need add a collision event to object, default is false
		 */
		public function set collisionCallbackOn(v : Boolean) : void {
			CModule.write8(pointer + 247, v ? 1 : 0);
		}

		/**
		 * set time step and simulate the physics world
		 * refer to: http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_the_World
		 */
		public function step(timeStep : Number, maxSubSteps : int = 1, fixedTimeStep : Number = 1.0 / 60) : void {
			physicsStepInC(timeStep, maxSubSteps, fixedTimeStep);

			for each (var body:AWPRigidBody in m_nonStaticRigidBodies) {
				body.updateTransform();
			}

			for each (var vehicle:AWPRaycastVehicle in m_vehicles) {
				vehicle.updateWheelsTransform();
			}

			for each (var character:AWPKinematicCharacterController in m_characters) {
				character.updateTransform();
			}
		}
	}
}
