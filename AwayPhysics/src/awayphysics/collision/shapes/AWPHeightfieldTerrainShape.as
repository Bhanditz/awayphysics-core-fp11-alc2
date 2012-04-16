package awayphysics.collision.shapes {
	import com.adobe.alchemy.CModule;
	import C_Run.createHeightmapDataBufferInC;
	import C_Run.removeHeightmapDataBufferInC;
	import C_Run.createTerrainShapeInC;
	import away3d.core.base.Geometry;
	import awayphysics.extend.AWPTerrain;

	public class AWPHeightfieldTerrainShape extends AWPCollisionShape {
		private var dataPtr : uint;

		private var _geometry:Geometry;
		
		 /**
		  * create terrain with the heightmap data
		  */
		public function AWPHeightfieldTerrainShape(terrain : AWPTerrain) {
			_geometry = terrain.geometry;
			var dataLen : int = terrain.sw * terrain.sh;
			dataPtr = createHeightmapDataBufferInC(dataLen);

			var data : Vector.<Number> = terrain.heights;
			for (var i : int = 0; i < dataLen; i++ ) {
				CModule.writeFloat(dataPtr+i*4,data[i] / _scaling);
			}

			pointer = createTerrainShapeInC(dataPtr, terrain.sw, terrain.sh, terrain.lw / _scaling, terrain.lh / _scaling, 1, -terrain.maxHeight / _scaling, terrain.maxHeight / _scaling, 1);
			super(pointer, 10);
		}

		/**
		 * release the heightmap data buffer
		 */
		public function deleteHeightfieldTerrainShapeBuffer() : void {
			removeHeightmapDataBufferInC(dataPtr);
		}
		
		public function get geometry():Geometry {
			return _geometry;
		}
	}
}