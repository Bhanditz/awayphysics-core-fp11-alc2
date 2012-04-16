package awayphysics.collision.dispatch {
	import C_Run.addCollisionObjectInC;
	import C_Run.removeCollisionObjectInC;
	
	import awayphysics.AWPBase;
	import awayphysics.collision.dispatch.AWPCollisionObject;
	import awayphysics.collision.shapes.AWPBvhTriangleMeshShape;
	import awayphysics.collision.shapes.AWPCompoundShape;
	import awayphysics.collision.shapes.AWPConvexHullShape;
	import awayphysics.collision.shapes.AWPHeightfieldTerrainShape;
	import awayphysics.data.AWPCollisionShapeType;
	
	import flash.utils.Dictionary;
		
	public class AWPCollisionWorld extends AWPBase{
		
		protected var m_collisionObjects:Dictionary;
		
		public function AWPCollisionWorld(){
			m_collisionObjects =  new Dictionary(true);
		}
		
		public function get collisionObjects() : Dictionary {
			return m_collisionObjects;
		}
		
		public function addCollisionObject(obj:AWPCollisionObject, group:int = 1, mask:int = -1):void{
			addCollisionObjectInC(obj.pointer, group, mask);
			
			if(!m_collisionObjects.hasOwnProperty(obj.pointer.toString())){
				m_collisionObjects[obj.pointer.toString()] = obj;
			}
		}
		
		public function removeCollisionObject(obj:AWPCollisionObject) : void {
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
			
			if(m_collisionObjects.hasOwnProperty(obj.pointer.toString())){
				delete m_collisionObjects[obj.pointer.toString()];
			}
		}
	}
}