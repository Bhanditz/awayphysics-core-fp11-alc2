package awayphysics.collision.shapes
{
	import AWPC_Run.CModule;
	import AWPC_Run.createTriangleVertexDataBufferInC;
	import AWPC_Run.removeTriangleVertexDataBufferInC;
	import AWPC_Run.createConvexHullShapeInC;
	import AWPC_Run.disposeCollisionShapeInC
	import away3d.core.base.Geometry;
	
	public class AWPConvexHullShape extends AWPCollisionShape
	{
		private var vertexDataPtr : uint;
		
		private var _geometry:Geometry;
		
		public function AWPConvexHullShape(geometry : Geometry)
		{
			_geometry = geometry;
			var vertexData : Vector.<Number> = geometry.subGeometries[0].vertexData;
			var vertexDataLen : int = vertexData.length;
			vertexDataPtr = createTriangleVertexDataBufferInC(vertexDataLen);
			
			for (var i:int = 0; i < vertexDataLen; i++ ) {
				CModule.writeFloat(vertexDataPtr+i*4,vertexData[i] / _scaling);
			}
			
			pointer = createConvexHullShapeInC(int(vertexDataLen / 3), vertexDataPtr);
			super(pointer, 5);
		}
		
		override public function dispose() : void
		{
			m_counter--;
			if (m_counter > 0) {
				return;
			}else {
				m_counter = 0;
			}
			if (!_cleanup) {
				_cleanup  = true;
				removeTriangleVertexDataBufferInC(vertexDataPtr);
				disposeCollisionShapeInC(pointer);
			}
		}
		
		public function get geometry():Geometry {
			return _geometry;
		}
	}
}