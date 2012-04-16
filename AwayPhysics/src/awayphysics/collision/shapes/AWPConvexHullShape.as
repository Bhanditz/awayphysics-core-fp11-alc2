package awayphysics.collision.shapes
{
	import com.adobe.alchemy.CModule;
	import C_Run.createTriangleVertexDataBufferInC;
	import C_Run.removeTriangleVertexDataBufferInC;
	import C_Run.createConvexHullShapeInC;
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
		
		public function deleteConvexHullShapeBuffer() : void
		{
			removeTriangleVertexDataBufferInC(vertexDataPtr);
		}
		
		public function get geometry():Geometry {
			return _geometry;
		}
	}
}