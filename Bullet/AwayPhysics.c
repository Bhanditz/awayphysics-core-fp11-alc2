
/*
compile: g++ -I./ AwayPhysics.c libbulletdynamics.a libbulletcollision.a libbulletmath.a -emit-swc -O4 -Wall -o AwayPhysics.swc
*/

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "AS3.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
//#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"


struct RayInfo
{
	RayInfo(btCollisionObject* collisionObject, const btVector3& rayFromLocal, const btVector3&	rayToLocal)
	:m_collisionObject(collisionObject),
	m_rayFromLocal(rayFromLocal),
	m_rayToLocal(rayToLocal)
	{
	}
	btCollisionObject* m_collisionObject;
	btVector3 m_rayFromLocal;
	btVector3 m_rayToLocal;
};
btAlignedObjectArray<RayInfo*> rays;

void vector3() __attribute__((used, annotate("as3sig:public function vector3():uint")));
void vector3() {
	btVector3* vect = new btVector3();
	AS3_Return(vect);
}
void matrix3x3() __attribute__((used, annotate("as3sig:public function matrix3x3():uint")));
void matrix3x3() {
	btMatrix3x3* mat = new btMatrix3x3();
	AS3_Return(mat);
}

btCollisionWorld* collisionWorld;

/// create the discrete dynamics world with btDbvtBroadphase
void createDiscreteDynamicsWorldWithDbvtInC() __attribute__((used, annotate("as3sig:public function createDiscreteDynamicsWorldWithDbvtInC():uint")));
void createDiscreteDynamicsWorldWithDbvtInC() {
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	collisionWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

	AS3_Return(collisionWorld);
}

/// create the discrete dynamics world with btAxisSweep3
void createDiscreteDynamicsWorldWithAxisSweep3InC() __attribute__((used, annotate("as3sig:public function createDiscreteDynamicsWorldWithAxisSweep3InC(as3_worldMin:uint,as3_worldMax:uint):uint")));
void createDiscreteDynamicsWorldWithAxisSweep3InC() {
	btVector3* worldMin;
	btVector3* worldMax;
	AS3_CopyAS3ToC(as3_worldMin,worldMin);
	AS3_CopyAS3ToC(as3_worldMax,worldMax);

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	btBroadphaseInterface* overlappingPairCache = new btAxisSweep3(*worldMin,*worldMax);
	overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	collisionWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

	AS3_Return(collisionWorld);
}

// create a static plane shape
void createStaticPlaneShapeInC() __attribute__((used, annotate("as3sig:public function createStaticPlaneShapeInC(as3_normal:uint,as3_constant:Number):uint")));
void createStaticPlaneShapeInC(){
	btVector3* normal;
	float constant;
	AS3_CopyAS3ToC(as3_normal,normal);
	AS3_CopyAS3ToC(as3_constant,constant);
	
	btCollisionShape* shape = new btStaticPlaneShape(*normal,constant);

	AS3_Return(shape);
}

// create a cube
void createBoxShapeInC() __attribute__((used, annotate("as3sig:public function createBoxShapeInC(as3_extents:uint):uint")));
void createBoxShapeInC(){
	btVector3* extents;
	AS3_CopyAS3ToC(as3_extents,extents);

	btCollisionShape* shape = new btBoxShape(btVector3(extents->m_floats[0]/2,extents->m_floats[1]/2,extents->m_floats[2]/2));

	AS3_Return(shape);
}

// create a sphere
void createSphereShapeInC() __attribute__((used, annotate("as3sig:public function createSphereShapeInC(as3_radius:Number):uint")));
void createSphereShapeInC(){
	float radius;
	AS3_CopyAS3ToC(as3_radius,radius);

	btCollisionShape* shape =  new btSphereShape(radius);

	AS3_Return(shape);
}

// create a cylinder
void createCylinderShapeInC() __attribute__((used, annotate("as3sig:public function createCylinderShapeInC(as3_extents:uint):uint")));
void createCylinderShapeInC(){
	btVector3* extents;
	AS3_CopyAS3ToC(as3_extents,extents);

	btCollisionShape* shape =  new btCylinderShape(btVector3(extents->m_floats[0]/2,extents->m_floats[1]/2,extents->m_floats[2]/2));
	
	AS3_Return(shape);
}

// create a capsule
void createCapsuleShapeInC() __attribute__((used, annotate("as3sig:public function createCapsuleShapeInC(as3_radius:Number,as3_height:Number):uint")));
void createCapsuleShapeInC(){
	float radius,height;
	AS3_CopyAS3ToC(as3_radius,radius);
	AS3_CopyAS3ToC(as3_height,height);
	
	btCollisionShape* shape =  new btCapsuleShape(radius,height);
	
	AS3_Return(shape);
}

// create a cone
void createConeShapeInC() __attribute__((used, annotate("as3sig:public function createConeShapeInC(as3_radius:Number,as3_height:Number):uint")));
void createConeShapeInC(){
	float radius,height;
	AS3_CopyAS3ToC(as3_radius,radius);
	AS3_CopyAS3ToC(as3_height,height);
	
	btCollisionShape* shape =  new btConeShape(radius,height);
	
	AS3_Return(shape);
}

// create a compound shape
void createCompoundShapeInC() __attribute__((used, annotate("as3sig:public function createCompoundShapeInC():uint")));
void createCompoundShapeInC(){

	btCollisionShape* shape =  new btCompoundShape();
	
	AS3_Return(shape);
}

//add a child shape to compound shape
void addCompoundChildInC() __attribute__((used, annotate("as3sig:public function addCompoundChildInC(as3_cshape:uint,as3_shape:uint,as3_pos:uint,as3_col:uint):uint")));
void addCompoundChildInC(){
	btCompoundShape* cshape;
	btCollisionShape* shape;
	btVector3* pos;
	btMatrix3x3* col;
	
	AS3_CopyAS3ToC(as3_cshape,cshape);
	AS3_CopyAS3ToC(as3_shape,shape);
	AS3_CopyAS3ToC(as3_pos,pos);
	AS3_CopyAS3ToC(as3_col,col);

	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(*pos);
	localTrans.setBasis(*col);

	cshape->addChildShape(localTrans,shape);

	AS3_Return(0);
}

//remove a child shape from compound shape by index
void removeCompoundChildInC() __attribute__((used, annotate("as3sig:public function removeCompoundChildInC(as3_cshape:uint,as3_index:int):uint")));
void removeCompoundChildInC(){
	btCompoundShape* cshape;
	int index;
	AS3_CopyAS3ToC(as3_cshape,cshape);
	AS3_CopyAS3ToC(as3_index,index);
	
	cshape->removeChildShapeByIndex(index);

	
	AS3_Return(0);
}

void createHeightmapDataBufferInC() __attribute__((used, annotate("as3sig:public function createHeightmapDataBufferInC(as3_size:int):uint")));
void createHeightmapDataBufferInC(){
	int size;
	AS3_CopyAS3ToC(as3_size,size);

	btScalar* heightmapData = new btScalar[size];

	AS3_Return(heightmapData);
}

void removeHeightmapDataBufferInC() __attribute__((used, annotate("as3sig:public function removeHeightmapDataBufferInC(as3_heightmapData:uint):uint")));
void removeHeightmapDataBufferInC(){
	btScalar* heightmapData;
	AS3_CopyAS3ToC(as3_heightmapData,heightmapData);

	delete [] heightmapData;

	AS3_Return(0);
}

void createTerrainShapeInC() __attribute__((used, annotate("as3sig:public function createTerrainShapeInC(as3_heightmapData:uint,as3_sw:int,as3_sh:int,as3_width:Number,as3_length:Number,as3_heightScale:Number,as3_minHeight:Number,as3_maxHeight:Number,as3_flipQuadEdges:int):uint")));
void createTerrainShapeInC(){
	btScalar* heightmapData;
	int sw,sh,flipQuadEdges;
	float width,length,heightScale,minHeight,maxHeight;
	
	AS3_CopyAS3ToC(as3_heightmapData,heightmapData);
	AS3_CopyAS3ToC(as3_sw,sw);
	AS3_CopyAS3ToC(as3_sh,sh);
	AS3_CopyAS3ToC(as3_flipQuadEdges,flipQuadEdges);
	AS3_CopyAS3ToC(as3_width,width);
	AS3_CopyAS3ToC(as3_length,length);
	AS3_CopyAS3ToC(as3_heightScale,heightScale);
	AS3_CopyAS3ToC(as3_minHeight,minHeight);
	AS3_CopyAS3ToC(as3_maxHeight,maxHeight);

	btHeightfieldTerrainShape* heightFieldShape = new btHeightfieldTerrainShape(sw,sh,heightmapData, heightScale,minHeight, maxHeight,1, PHY_FLOAT,flipQuadEdges==1);
	heightFieldShape->setUseDiamondSubdivision(true);
	heightFieldShape->setLocalScaling(btVector3(width/sw,1,length/sh));
	
	AS3_Return(heightFieldShape);
}

void createTriangleIndexDataBufferInC() __attribute__((used, annotate("as3sig:public function createTriangleIndexDataBufferInC(as3_size:int):uint")));
void createTriangleIndexDataBufferInC(){
	int size;
	AS3_CopyAS3ToC(as3_size,size);

	int* indexData = new int[size];

	AS3_Return(indexData);
}

void removeTriangleIndexDataBufferInC() __attribute__((used, annotate("as3sig:public function removeTriangleIndexDataBufferInC(as3_indexData:uint):uint")));
void removeTriangleIndexDataBufferInC(){
	int* indexData;
	AS3_CopyAS3ToC(as3_indexData,indexData);

	delete [] indexData;

	AS3_Return(0);
}

void createTriangleVertexDataBufferInC() __attribute__((used, annotate("as3sig:public function createTriangleVertexDataBufferInC(as3_size:int):uint")));
void createTriangleVertexDataBufferInC(){
	int size;
	AS3_CopyAS3ToC(as3_size,size);

	btScalar* vertexData = new btScalar[size];

	AS3_Return(vertexData);
}

void removeTriangleVertexDataBufferInC() __attribute__((used, annotate("as3sig:public function removeTriangleVertexDataBufferInC(as3_vertexData:uint):uint")));
void removeTriangleVertexDataBufferInC(){
	btScalar* vertexData;
	AS3_CopyAS3ToC(as3_vertexData,vertexData);

	delete [] vertexData;

	AS3_Return(0);
}

void createTriangleIndexVertexArrayInC() __attribute__((used, annotate("as3sig:public function createTriangleIndexVertexArrayInC(as3_numTriangles:int,as3_indexBase:uint,as3_numVertices:int,as3_vertexBase:uint):uint")));
void createTriangleIndexVertexArrayInC(){
	int numTriangles;
	int* indexBase;
	int numVertices;
	btScalar* vertexBase;
	AS3_CopyAS3ToC(as3_numTriangles,numTriangles);
	AS3_CopyAS3ToC(as3_indexBase,indexBase);
	AS3_CopyAS3ToC(as3_numVertices,numVertices);
	AS3_CopyAS3ToC(as3_vertexBase,vertexBase);

	int indexStride = 3*sizeof(int);
	int vertStride = 3*sizeof(btScalar);

	btTriangleIndexVertexArray* indexVertexArrays=new btTriangleIndexVertexArray(numTriangles,indexBase,indexStride,numVertices,vertexBase,vertStride);

	AS3_Return(indexVertexArrays);
}

void createBvhTriangleMeshShapeInC() __attribute__((used, annotate("as3sig:public function createBvhTriangleMeshShapeInC(as3_indexVertexArrays:uint,as3_useQuantizedAabbCompression:int,as3_buildBvh:int):uint")));
void createBvhTriangleMeshShapeInC(){
	btTriangleIndexVertexArray* indexVertexArrays;
	int useQuantizedAabbCompression;
	int buildBvh;
	AS3_CopyAS3ToC(as3_indexVertexArrays,indexVertexArrays);
	AS3_CopyAS3ToC(as3_useQuantizedAabbCompression,useQuantizedAabbCompression);
	AS3_CopyAS3ToC(as3_buildBvh,buildBvh);

	btBvhTriangleMeshShape* bvhTriangleMesh=new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression==1,buildBvh==1);

	AS3_Return(bvhTriangleMesh);
}

void createConvexHullShapeInC() __attribute__((used, annotate("as3sig:public function createConvexHullShapeInC(as3_numPoints:int,as3_points:uint):uint")));
void createConvexHullShapeInC(){
	int numPoints;
	btScalar* points;
	AS3_CopyAS3ToC(as3_numPoints,numPoints);
	AS3_CopyAS3ToC(as3_points,points);

	btConvexHullShape* convexHullShape=new btConvexHullShape(points, numPoints, sizeof(btScalar) * 3);

	AS3_Return(convexHullShape);
}
/*
void createGImpactMeshShapeInC() __attribute__((used, annotate("as3sig:public function createGImpactMeshShapeInC(as3_indexVertexArrays:uint):uint")));
void createGImpactMeshShapeInC(){
	btTriangleIndexVertexArray* indexVertexArrays;
	AS3_CopyAS3ToC(as3_indexVertexArrays,indexVertexArrays);

	btGImpactMeshShape* gimpactMesh = new btGImpactMeshShape(indexVertexArrays);
	gimpactMesh->updateBound();

	btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(collisionWorld ->getDispatcher());
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

	AS3_Return(gimpactMesh);
}
*/
void createTriangleShapeInC() __attribute__((used, annotate("as3sig:public function createTriangleShapeInC(as3_p0:uint,as3_p1:uint,as3_p2:uint):uint")));
void createTriangleShapeInC(){
	btVector3* p0;
	btVector3* p1;
	btVector3* p2;
	AS3_CopyAS3ToC(as3_p0,p0);
	AS3_CopyAS3ToC(as3_p1,p1);
	AS3_CopyAS3ToC(as3_p2,p2);

	btTriangleShapeEx* triangleShape=new btTriangleShapeEx(*p0,*p1,*p2);

	AS3_Return(triangleShape);
}

void setShapeScalingInC() __attribute__((used, annotate("as3sig:public function setShapeScalingInC(as3_shape:uint,as3_scale:uint):uint")));
void setShapeScalingInC(){
	btCollisionShape* shape;
	btVector3* scale;
	AS3_CopyAS3ToC(as3_shape,shape);
	AS3_CopyAS3ToC(as3_scale,scale);
	
	shape->setLocalScaling(*scale);

	AS3_Return(0);
}

void createCollisionObjectInC() __attribute__((used, annotate("as3sig:public function createCollisionObjectInC(as3_shape:uint):uint")));
void createCollisionObjectInC(){
	btCollisionShape* shape;
	AS3_CopyAS3ToC(as3_shape,shape);
	
	btCollisionObject* obj = new btCollisionObject();
	obj->setCollisionShape(shape);
	
	AS3_Return(obj);
}
void addCollisionObjectInC() __attribute__((used, annotate("as3sig:public function addCollisionObjectInC(as3_obj:uint,as3_group:int,as3_mask:int):uint")));
void addCollisionObjectInC(){
	btCollisionObject* obj;
	int group;
	int mask;
	AS3_CopyAS3ToC(as3_obj,obj);
	AS3_CopyAS3ToC(as3_group,group);
	AS3_CopyAS3ToC(as3_mask,mask);

	collisionWorld->addCollisionObject(obj,group,mask);
	
	AS3_Return(0);
}
void removeCollisionObjectInC() __attribute__((used, annotate("as3sig:public function removeCollisionObjectInC(as3_obj:uint):uint")));
void removeCollisionObjectInC(){
	btCollisionObject* obj;
	AS3_CopyAS3ToC(as3_obj,obj);

	collisionWorld->removeCollisionObject(obj);

	AS3_Return(0);
}

void addRayInC() __attribute__((used, annotate("as3sig:public function addRayInC(as3_obj:uint,as3_from:uint,as3_to:uint):uint")));
void addRayInC(){
	btCollisionObject* obj;
	btVector3* from;
	btVector3* to;
	AS3_CopyAS3ToC(as3_obj,obj);
	AS3_CopyAS3ToC(as3_from,from);
	AS3_CopyAS3ToC(as3_to,to);
	
	RayInfo* ray=new RayInfo(obj,*from,*to);
	rays.push_back(ray);
	
	AS3_Return(ray);
}
void removeRayInC() __attribute__((used, annotate("as3sig:public function removeRayInC(as3_ray:uint):uint")));
void removeRayInC(){
	RayInfo* ray;
	AS3_CopyAS3ToC(as3_ray,ray);
	
	rays.remove(ray);
	
	
	delete ray;
	
	AS3_Return(0);
}

// create rigidbody
void createBodyInC() __attribute__((used, annotate("as3sig:public function createBodyInC(as3_shape:uint,as3_mass:Number):uint")));
void createBodyInC(){
	btCollisionShape* shape;
	float mass;
	
	AS3_CopyAS3ToC(as3_shape,shape);
	AS3_CopyAS3ToC(as3_mass,mass);
	
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState();
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	AS3_Return(body);
}

void setBodyMassInC() __attribute__((used, annotate("as3sig:public function setBodyMassInC(as3_body:uint,as3_mass:Number):uint")));
void setBodyMassInC(){
	btRigidBody* body;
	float mass;
	AS3_CopyAS3ToC(as3_body,body);
	AS3_CopyAS3ToC(as3_mass,mass);	
	
	
	btCollisionShape* shape=body->getCollisionShape();
	
	
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);
		
	
	body->setMassProps(mass, localInertia);
	
	body->updateInertiaTensor();
	
	btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)collisionWorld;
	if(dynamicsWorld->getCollisionObjectArray().findLinearSearch(body) != dynamicsWorld->getNumCollisionObjects()){
		short int group = body->getBroadphaseHandle()->m_collisionFilterGroup;
		short int mask = body->getBroadphaseHandle()->m_collisionFilterMask;
		dynamicsWorld->removeRigidBody(body);
		dynamicsWorld->addRigidBody(body,group,mask);
	}
	AS3_Return(0);
}

//add the body to the dynamics world
void addBodyInC() __attribute__((used, annotate("as3sig:public function addBodyInC(as3_body:uint):uint")));
void addBodyInC(){
	btRigidBody* body;
	AS3_CopyAS3ToC(as3_body,body);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->addRigidBody(body);

	AS3_Return(0);
}

//add a body to the dynamics world with group and mask
void addBodyWithGroupInC() __attribute__((used, annotate("as3sig:public function addBodyWithGroupInC(as3_body:uint,as3_group:int,as3_mask:int):uint")));
void addBodyWithGroupInC(){
	btRigidBody* body;
	int group;
	int mask;
	AS3_CopyAS3ToC(as3_body,body);
	AS3_CopyAS3ToC(as3_group,group);
	AS3_CopyAS3ToC(as3_mask,mask);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->addRigidBody(body,group,mask);

	AS3_Return(0);
}

/// remove rigidbody
void removeBodyInC() __attribute__((used, annotate("as3sig:public function removeBodyInC(as3_body:uint):uint")));
void removeBodyInC(){
	btRigidBody* body;
	AS3_CopyAS3ToC(as3_body,body);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->removeRigidBody(body);

	
	AS3_Return(0);
}

//create a btPoint2PointConstraint with one rigidbody
void createP2PConstraint1InC() __attribute__((used, annotate("as3sig:public function createP2PConstraint1InC(as3_bodyA:uint,as3_pivotInA:uint):uint")));
void createP2PConstraint1InC(){
	btRigidBody* bodyA;
	btVector3* pivotInA;
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);

	btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*bodyA,*pivotInA);

	AS3_Return(p2p);
}

//create a btPoint2PointConstraint between tow rigidbodies
void createP2PConstraint2InC() __attribute__((used, annotate("as3sig:public function createP2PConstraint2InC(as3_bodyA:uint,as3_bodyB:uint,as3_pivotInA:uint,as3_pivotInB:uint):uint")));
void createP2PConstraint2InC(){
	btRigidBody* bodyA;
	btRigidBody* bodyB;
	btVector3* pivotInA;
	btVector3* pivotInB;
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_bodyB,bodyB);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);
	AS3_CopyAS3ToC(as3_pivotInB,pivotInB);

	btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*bodyA,*bodyB,*pivotInA,*pivotInB);

	AS3_Return(p2p);
}

void createHingeConstraint1InC() __attribute__((used, annotate("as3sig:public function createHingeConstraint1InC(as3_bodyA:uint,as3_pivotInA:uint,as3_axisInA:uint,as3_useReferenceFrameA:int):uint")));
void createHingeConstraint1InC(){
	btRigidBody* bodyA;
	btVector3* pivotInA;
	btVector3* axisInA;
	int useReferenceFrameA;
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);
	AS3_CopyAS3ToC(as3_axisInA,axisInA);
	AS3_CopyAS3ToC(as3_useReferenceFrameA,useReferenceFrameA);

	btHingeConstraint* hinge = new btHingeConstraint(*bodyA, *pivotInA, *axisInA,useReferenceFrameA==1);

	AS3_Return(hinge);
}

void createHingeConstraint2InC() __attribute__((used, annotate("as3sig:public function createHingeConstraint2InC(as3_bodyA:uint,as3_bodyB:uint,as3_pivotInA:uint,as3_pivotInB:uint,as3_axisInA:uint,as3_axisInB:uint,as3_useReferenceFrameA:int):uint")));
void createHingeConstraint2InC(){
	btRigidBody* bodyA;
	btRigidBody* bodyB;
	btVector3* pivotInA;
	btVector3* pivotInB;
	btVector3* axisInA;
	btVector3* axisInB;
	int useReferenceFrameA;
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_bodyB,bodyB);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);
	AS3_CopyAS3ToC(as3_pivotInB,pivotInB);
	AS3_CopyAS3ToC(as3_axisInA,axisInA);
	AS3_CopyAS3ToC(as3_axisInB,axisInB);
	AS3_CopyAS3ToC(as3_useReferenceFrameA,useReferenceFrameA);

	btHingeConstraint* hinge = new btHingeConstraint(*bodyA,*bodyB, *pivotInA,*pivotInB, *axisInA,*axisInB,useReferenceFrameA==1);

	AS3_Return(hinge);
}

void createConeTwistConstraint1() __attribute__((used, annotate("as3sig:public function createConeTwistConstraint1(as3_bodyA:uint,as3_pivotInA:uint,as3_rot:uint):uint")));
void createConeTwistConstraint1(){
	btRigidBody* bodyA;
	btVector3* pivotInA;
	btMatrix3x3* rot;
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);
	AS3_CopyAS3ToC(as3_rot,rot);

	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(*pivotInA);

	frameInA.setBasis(*rot);

	btConeTwistConstraint* coneTwist=new btConeTwistConstraint(*bodyA,frameInA);

	AS3_Return(coneTwist);
}

void createConeTwistConstraint2() __attribute__((used, annotate("as3sig:public function createConeTwistConstraint2(as3_bodyA:uint,as3_pivotInA:uint,as3_rotationInA:uint,as3_bodyB:uint,as3_pivotInB:uint,as3_rotationInB:uint):uint")));
void createConeTwistConstraint2(){
	btRigidBody* bodyA;
	btVector3* pivotInA;
	btMatrix3x3* rotationInA;
	btRigidBody* bodyB;
	btVector3* pivotInB;
	btMatrix3x3* rotationInB;
	
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);
	AS3_CopyAS3ToC(as3_rotationInA,rotationInA);
	AS3_CopyAS3ToC(as3_bodyB,bodyB);
	AS3_CopyAS3ToC(as3_pivotInB,pivotInB);
	AS3_CopyAS3ToC(as3_rotationInB,rotationInB);

	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(*pivotInA);
	frameInA.setBasis(*rotationInA);

	btTransform frameInB;
	frameInB.setIdentity();
	frameInB.setOrigin(*pivotInB);
	frameInB.setBasis(*rotationInB);

	btConeTwistConstraint* coneTwist=new btConeTwistConstraint(*bodyA,*bodyB,frameInA,frameInB);

	AS3_Return(coneTwist);
}

void createGeneric6DofConstraint1() __attribute__((used, annotate("as3sig:public function createGeneric6DofConstraint1(as3_bodyA:uint,as3_pivotInA:uint,as3_rot:uint,as3_useLinearReferenceFrameA:int):uint")));
void createGeneric6DofConstraint1(){
	btRigidBody* bodyA;
	btVector3* pivotInA;
	btMatrix3x3* rot;
	int useLinearReferenceFrameA;
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);
	AS3_CopyAS3ToC(as3_rot,rot);
	AS3_CopyAS3ToC(as3_useLinearReferenceFrameA,useLinearReferenceFrameA);

	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(*pivotInA);
	frameInA.setBasis(*rot);

	btGeneric6DofConstraint* generic6Dof=new btGeneric6DofConstraint(*bodyA,frameInA,useLinearReferenceFrameA==1);

	AS3_Return(generic6Dof);
}

void createGeneric6DofConstraint2() __attribute__((used, annotate("as3sig:public function createGeneric6DofConstraint2(as3_bodyA:uint,as3_pivotInA:uint,as3_rotationInA:uint,as3_bodyB:uint,as3_pivotInB:uint,as3_rotationInB:uint,as3_useLinearReferenceFrameA:int):uint")));
void createGeneric6DofConstraint2(){
	btRigidBody* bodyA;
	btVector3* pivotInA;
	btMatrix3x3* rotationInA;
	btRigidBody* bodyB;
	btVector3* pivotInB;
	btMatrix3x3* rotationInB;
	int useLinearReferenceFrameA;
	
	AS3_CopyAS3ToC(as3_bodyA,bodyA);
	AS3_CopyAS3ToC(as3_pivotInA,pivotInA);
	AS3_CopyAS3ToC(as3_rotationInA,rotationInA);
	AS3_CopyAS3ToC(as3_bodyB,bodyB);
	AS3_CopyAS3ToC(as3_pivotInB,pivotInB);
	AS3_CopyAS3ToC(as3_rotationInB,rotationInB);
	AS3_CopyAS3ToC(as3_useLinearReferenceFrameA,useLinearReferenceFrameA);
	
	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(*pivotInA);
	frameInA.setBasis(*rotationInA);

	btTransform frameInB;
	frameInB.setIdentity();
	frameInB.setOrigin(*pivotInB);
	frameInB.setBasis(*rotationInB);

	btGeneric6DofConstraint* generic6Dof=new btGeneric6DofConstraint(*bodyA,*bodyB,frameInA,frameInB,useLinearReferenceFrameA==1);

	AS3_Return(generic6Dof);
}

//add a constraint to the dynamics world
void addConstraintInC() __attribute__((used, annotate("as3sig:public function addConstraintInC(as3_constraint:uint,as3_disableCollisions:int):uint")));
void addConstraintInC(){
	btTypedConstraint* constraint;
	int disableCollisions;
	AS3_CopyAS3ToC(as3_constraint,constraint);
	AS3_CopyAS3ToC(as3_disableCollisions,disableCollisions);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->addConstraint(constraint,disableCollisions==1);
	
	AS3_Return(0);
}

/// remove constraint
void removeConstraintInC() __attribute__((used, annotate("as3sig:public function removeConstraintInC(as3_constraint:uint):uint")));
void removeConstraintInC(){
	btTypedConstraint* constraint;
	AS3_CopyAS3ToC(as3_constraint,constraint);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->removeConstraint(constraint);

	
	AS3_Return(0);
}

void createVehicleInC() __attribute__((used, annotate("as3sig:public function createVehicleInC(as3_chassis:uint,as3_suspensionStiffness:Number,as3_suspensionCompression:Number,as3_suspensionDamping:Number,as3_maxSuspensionTravelCm:Number,as3_frictionSlip:Number,as3_maxSuspensionForce:Number):uint")));
void createVehicleInC() {
	btRigidBody* chassis;
	float suspensionStiffness;
	float suspensionCompression;
	float suspensionDamping;
	float maxSuspensionTravelCm;
	float frictionSlip;
	float maxSuspensionForce;
	AS3_CopyAS3ToC(as3_chassis,chassis);
	AS3_CopyAS3ToC(as3_suspensionStiffness,suspensionStiffness);
	AS3_CopyAS3ToC(as3_suspensionCompression,suspensionCompression);
	AS3_CopyAS3ToC(as3_suspensionDamping,suspensionDamping);
	AS3_CopyAS3ToC(as3_maxSuspensionTravelCm,maxSuspensionTravelCm);
	AS3_CopyAS3ToC(as3_frictionSlip,frictionSlip);
	AS3_CopyAS3ToC(as3_maxSuspensionForce,maxSuspensionForce);

	btRaycastVehicle::btVehicleTuning m_tuning;
	m_tuning.m_suspensionStiffness=suspensionStiffness;
	m_tuning.m_suspensionCompression=suspensionCompression;
	m_tuning.m_suspensionDamping=suspensionDamping;
	m_tuning.m_maxSuspensionTravelCm=maxSuspensionTravelCm;
	m_tuning.m_frictionSlip=frictionSlip;
	m_tuning.m_maxSuspensionForce=maxSuspensionForce;

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	btVehicleRaycaster*	m_vehicleRayCaster = new btDefaultVehicleRaycaster(dynamicsWorld);
	btRaycastVehicle* m_vehicle = new btRaycastVehicle(m_tuning,chassis,m_vehicleRayCaster);
	m_vehicle->setCoordinateSystem(0,1,2);

	AS3_Return(m_vehicle);
}

void addVehicleWheelInC() __attribute__((used, annotate("as3sig:public function addVehicleWheelInC(as3_vehicle:uint,as3_connectionPointCS0:uint,as3_wheelDirectionCS0:uint,as3_wheelAxleCS:uint,as3_suspensionStiffness:Number,as3_suspensionCompression:Number,as3_suspensionDamping:Number,as3_maxSuspensionTravelCm:Number,as3_frictionSlip:Number,as3_maxSuspensionForce:Number,as3_suspensionRestLength:Number,as3_wheelRadius:Number,as3_isFrontWheel:int):uint")));
void addVehicleWheelInC(){
	btRaycastVehicle* m_vehicle;
	btVector3* connectionPointCS0;
	btVector3* wheelDirectionCS0;
	btVector3* wheelAxleCS;
	float suspensionStiffness;
	float suspensionCompression;
	float suspensionDamping;
	float maxSuspensionTravelCm;
	float frictionSlip;
	float maxSuspensionForce;
	float suspensionRestLength;
	float wheelRadius;
	int isFrontWheel;
	AS3_CopyAS3ToC(as3_vehicle,m_vehicle);
	AS3_CopyAS3ToC(as3_connectionPointCS0,connectionPointCS0);
	AS3_CopyAS3ToC(as3_wheelDirectionCS0,wheelDirectionCS0);
	AS3_CopyAS3ToC(as3_wheelAxleCS,wheelAxleCS);
	AS3_CopyAS3ToC(as3_suspensionStiffness,suspensionStiffness);
	AS3_CopyAS3ToC(as3_suspensionCompression,suspensionCompression);
	AS3_CopyAS3ToC(as3_suspensionDamping,suspensionDamping);
	AS3_CopyAS3ToC(as3_maxSuspensionTravelCm,maxSuspensionTravelCm);
	AS3_CopyAS3ToC(as3_frictionSlip,frictionSlip);
	AS3_CopyAS3ToC(as3_maxSuspensionForce,maxSuspensionForce);
	AS3_CopyAS3ToC(as3_suspensionRestLength,suspensionRestLength);
	AS3_CopyAS3ToC(as3_wheelRadius,wheelRadius);
	AS3_CopyAS3ToC(as3_isFrontWheel,isFrontWheel);

	btRaycastVehicle::btVehicleTuning m_tuning;
	m_tuning.m_suspensionStiffness=suspensionStiffness;
	m_tuning.m_suspensionCompression=suspensionCompression;
	m_tuning.m_suspensionDamping=suspensionDamping;
	m_tuning.m_maxSuspensionTravelCm=maxSuspensionTravelCm;
	m_tuning.m_frictionSlip=frictionSlip;
	m_tuning.m_maxSuspensionForce=maxSuspensionForce;

	m_vehicle->addWheel(*connectionPointCS0,*wheelDirectionCS0,*wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel==1);

	AS3_Return(&m_vehicle->getWheelInfo(m_vehicle->getNumWheels()-1));
}

void addVehicleInC() __attribute__((used, annotate("as3sig:public function addVehicleInC(as3_vehicle:uint):uint")));
void addVehicleInC(){
	btActionInterface* vehicle;
	AS3_CopyAS3ToC(as3_vehicle,vehicle);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->addVehicle(vehicle);

	AS3_Return(0);
}

void removeVehicleInC() __attribute__((used, annotate("as3sig:public function removeVehicleInC(as3_vehicle:uint):uint")));
void removeVehicleInC(){
	btActionInterface* vehicle;
	AS3_CopyAS3ToC(as3_vehicle,vehicle);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->removeVehicle(vehicle);
	
	AS3_Return(0);
}

void createGhostObjectInC() __attribute__((used, annotate("as3sig:public function createGhostObjectInC(as3_shape:uint):uint")));
void createGhostObjectInC(){
	btCollisionShape* shape;
	AS3_CopyAS3ToC(as3_shape,shape);

	btPairCachingGhostObject* ghostObject = new btPairCachingGhostObject();
	ghostObject->setCollisionShape(shape);

	AS3_Return(ghostObject);
}

void createCharacterInC() __attribute__((used, annotate("as3sig:public function createCharacterInC(as3_ghostObject:uint,as3_shape:uint,as3_stepHeight:Number,as3_upAxis:int):uint")));
void createCharacterInC(){
	btPairCachingGhostObject* ghostObject;
	btConvexShape* shape;
	float stepHeight;
	int upAxis;
	AS3_CopyAS3ToC(as3_ghostObject,ghostObject);
	AS3_CopyAS3ToC(as3_shape,shape);
	AS3_CopyAS3ToC(as3_stepHeight,stepHeight);
	AS3_CopyAS3ToC(as3_upAxis,upAxis);

	btKinematicCharacterController* character = new btKinematicCharacterController (ghostObject,shape,stepHeight,upAxis);

	AS3_Return(character);
}

void addCharacterInC() __attribute__((used, annotate("as3sig:public function addCharacterInC(as3_character:uint,as3_group:int,as3_mask:int):uint")));
void addCharacterInC(){
	btKinematicCharacterController* character;
	int group;
	int mask;
	AS3_CopyAS3ToC(as3_character,character);
	AS3_CopyAS3ToC(as3_group,group);
	AS3_CopyAS3ToC(as3_mask,mask);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->addCollisionObject(character->m_ghostObject,group,mask);
	dynamicsWorld->addCharacter(character);

	AS3_Return(0);
}

void removeCharacterInC() __attribute__((used, annotate("as3sig:public function removeCharacterInC(as3_character:uint):uint")));
void removeCharacterInC(){
	btKinematicCharacterController* character;
	AS3_CopyAS3ToC(as3_character,character);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->removeCollisionObject(character->m_ghostObject);
	
	dynamicsWorld->removeCharacter(character);
	
	AS3_Return(0);
}

/// physic step
void physicsStepInC() __attribute__((used, annotate("as3sig:public function physicsStepInC(as3_timestep:Number,as3_maxsubstep:int,as3_fixedtime:Number):uint")));
void physicsStepInC() {
	float timestep;
	int maxsubstep;
	float fixedtime;
	AS3_CopyAS3ToC(as3_timestep,timestep);
	AS3_CopyAS3ToC(as3_maxsubstep,maxsubstep);
	AS3_CopyAS3ToC(as3_fixedtime,fixedtime);

	btDiscreteDynamicsWorld* dynamicsWorld=(btDiscreteDynamicsWorld*)collisionWorld;
	dynamicsWorld->stepSimulation(timestep,maxsubstep,fixedtime);

	int vehiclesLen=dynamicsWorld->m_vehicles.size();
	for (int i=0;i<vehiclesLen;i++)
	{
		btRaycastVehicle* vehicle=(btRaycastVehicle*)dynamicsWorld->m_vehicles[i];
		int wheelLen=vehicle->getNumWheels();
		for (int j=0;j<wheelLen;j++){
			vehicle->updateWheelTransform(j,true);
		}
	}
	
	int rayLen = rays.size();
	for (int i=0;i<rayLen;i++)
	{
		RayInfo* ray = rays[i];
		btVector3 rayFrom = ray->m_collisionObject->m_worldTransform*ray->m_rayFromLocal;
		btVector3 rayTo = ray->m_collisionObject->m_worldTransform*ray->m_rayToLocal;
		btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom, rayTo);
		collisionWorld->rayTest(rayFrom,rayTo,resultCallback);
		if (resultCallback.hasHit()){
			btManifoldPoint* mpt=new btManifoldPoint();
			mpt->m_localPointA=rayFrom;
			mpt->m_localPointB=resultCallback.m_collisionObject->m_worldTransform.invXform(resultCallback.m_hitPointWorld);
			mpt->m_normalWorldOnB=resultCallback.m_hitNormalWorld;
			mpt->m_appliedImpulse=0;
			
			inline_as3(
				"import com.adobe.alchemy.AlcConsole;\n"
	  			"AlcConsole.current.rayCastCallback(%0,%1,%2);\n"
                : : "r"(ray->m_collisionObject), "r"(mpt), "r"(resultCallback.m_collisionObject)
            );
			
			delete mpt;
		}
	}

	if(dynamicsWorld->m_collisionCallbackOn){
		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

			if (obA->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK){
				int numContacts = contactManifold->getNumContacts();
				if(numContacts>0){
					btManifoldPoint* mpt=new btManifoldPoint();
					mpt->m_localPointA=btVector3(0,0,0);
					mpt->m_localPointB=btVector3(0,0,0);
					mpt->m_normalWorldOnB=btVector3(0,0,0);
					mpt->m_appliedImpulse=0;
					for (int j=0;j<numContacts;j++)
					{
						btManifoldPoint& pt = contactManifold->getContactPoint(j);
						mpt->m_localPointA+=pt.m_localPointA;
						mpt->m_localPointB+=pt.m_localPointB;
						mpt->m_normalWorldOnB+=pt.m_normalWorldOnB;
						mpt->m_appliedImpulse+=pt.m_appliedImpulse;
					}
					mpt->m_localPointA/=numContacts;
					mpt->m_localPointB/=numContacts;
					mpt->m_normalWorldOnB.normalize();
					mpt->m_appliedImpulse/=numContacts;
					
					inline_as3(
						"import com.adobe.alchemy.AlcConsole;\n"
	  					"AlcConsole.current.collisionCallback(%0,%1,%2);\n"
                		: : "r"(obA), "r"(mpt), "r"(obB)
           			);

					delete mpt;
				}
			}

			if(obB->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK){
				int numContacts = contactManifold->getNumContacts();
				if(numContacts>0){
					btManifoldPoint* mpt=new btManifoldPoint();
					mpt->m_localPointA=btVector3(0,0,0);
					mpt->m_localPointB=btVector3(0,0,0);
					mpt->m_normalWorldOnB=btVector3(0,0,0);
					mpt->m_appliedImpulse=0;
					for (int j=0;j<numContacts;j++)
					{
						btManifoldPoint& pt = contactManifold->getContactPoint(j);
						mpt->m_localPointA+=pt.m_localPointB;
						mpt->m_localPointB+=pt.m_localPointA;
						mpt->m_normalWorldOnB+=pt.m_normalWorldOnB;
						mpt->m_appliedImpulse+=pt.m_appliedImpulse;
					}
					mpt->m_localPointA/=numContacts;
					mpt->m_localPointB/=numContacts;
					mpt->m_normalWorldOnB/=-1;
					mpt->m_normalWorldOnB.normalize();
					mpt->m_appliedImpulse/=numContacts;
					
					inline_as3(
						"import com.adobe.alchemy.AlcConsole;\n"
	  					"AlcConsole.current.collisionCallback(%0,%1,%2);\n"
                		: : "r"(obB), "r"(mpt), "r"(obA)
           			);

					delete mpt;
				}
			}
		}
	}

	AS3_Return(0);
}

int main() {

	AS3_GoAsync();
	
	return 0;
}