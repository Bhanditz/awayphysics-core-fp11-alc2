package awayphysics.collision.shapes {
	import com.adobe.alchemy.CModule;
	import C_Run.createTriangleIndexDataBufferInC;
	import C_Run.removeTriangleIndexDataBufferInC;
	import C_Run.createTriangleVertexDataBufferInC;
	import C_Run.removeTriangleVertexDataBufferInC;
	import C_Run.createTriangleIndexVertexArrayInC;
	import C_Run.createBvhTriangleMeshShapeInC;
	
	import away3d.core.base.Geometry;

	public class AWPBvhTriangleMeshShape extends AWPCollisionShape {
		private var indexDataPtr : uint;
		private var vertexDataPtr : uint;
		
		private var _geometry:Geometry;

		/**
		 *create a static triangle mesh shape with a 3D mesh object
		 */
		public function AWPBvhTriangleMeshShape(geometry : Geometry, useQuantizedAabbCompression : Boolean = true) {
			_geometry = geometry;
			var indexData : Vector.<uint> = geometry.subGeometries[0].indexData;
			var indexDataLen : int = indexData.length;
			indexDataPtr = createTriangleIndexDataBufferInC(indexDataLen);

			for (var i : int = 0; i < indexDataLen; i++ ) {
				CModule.write32(indexDataPtr+i*4,indexData[i]);
			}

			var vertexData : Vector.<Number> = geometry.subGeometries[0].vertexData;
			var vertexDataLen : int = vertexData.length;
			vertexDataPtr = createTriangleVertexDataBufferInC(vertexDataLen);

			for (i = 0; i < vertexDataLen; i++ ) {
				CModule.writeFloat(vertexDataPtr+i*4,vertexData[i] / _scaling);
			}

			var triangleIndexVertexArrayPtr : uint = createTriangleIndexVertexArrayInC(int(indexDataLen / 3), indexDataPtr, int(vertexDataLen / 3), vertexDataPtr);

			pointer = createBvhTriangleMeshShapeInC(triangleIndexVertexArrayPtr, useQuantizedAabbCompression ? 1 : 0, 1);
			super(pointer, 9);
		}

		/**
		 *release the memory of index/vertex buffer
		 */
		public function deleteBvhTriangleMeshShapeBuffer() : void {
			removeTriangleIndexDataBufferInC(indexDataPtr);
			removeTriangleVertexDataBufferInC(vertexDataPtr);
		}
		
		public function get geometry():Geometry {
			return _geometry;
		}
	}
}