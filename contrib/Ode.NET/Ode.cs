using System;
using System.Runtime.InteropServices;

namespace Ode.NET
{
#if dSINGLE
	using dReal = System.Single;
#else
	using dReal = System.Double;
#endif

	public static class d
	{
		#region Type Definitions

		[StructLayout(LayoutKind.Sequential)]
		public struct Matrix3
		{
			public Matrix3(dReal m00, dReal m10, dReal m20, dReal m01, dReal m11, dReal m21, dReal m02, dReal m12, dReal m22)
			{
				M00 = m00;
				M10 = m10;
				M20 = m20;
				M01 = m01;
				M11 = m11;
				M21 = m21;
				M02 = m02;
				M12 = m12;
				M22 = m22;
				_pad30 = 0.0f;
				_pad31 = 0.0f;
				_pad32 = 0.0f;
			}

			public dReal M00, M10, M20;
			dReal _pad30;
			public dReal M01, M11, M21;
			dReal _pad31;
			public dReal M02, M12, M22;
			dReal _pad32;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Quaternion
		{
			public dReal W, X, Y, Z;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Vector3
		{
			public Vector3(dReal x, dReal y, dReal z)
			{
				X = x;
				Y = y;
				Z = z;
				_padding = 0.0f;
			}

			public dReal X, Y, Z;
			private dReal _padding;
		}

		#endregion

		[DllImport("ode", EntryPoint = "dBodyCopyPosition")]
		public static extern void BodyCopyPosition(IntPtr body, out Vector3 pos);

		[DllImport("ode", EntryPoint = "dBodyCopyQuaternion")]
		public static extern void BodyCopyQuaternion(IntPtr body, out Quaternion quat);

		[DllImport("ode", EntryPoint = "dBodyCreate")]
		public static extern IntPtr BodyCreate(IntPtr world);

		[DllImport("ode", EntryPoint = "dBodyDestroy")]
		public static extern void BodyDestroy(IntPtr body);

		[DllImport("ode", EntryPoint = "dBodySetPosition")]
		public static extern void BodySetPosition(IntPtr body, dReal x, dReal y, dReal z);

		[DllImport("ode", EntryPoint = "dCloseODE")]
		public static extern void CloseODE();

		[DllImport("ode", EntryPoint = "dCreateBox")]
		public static extern IntPtr CreateBox(IntPtr space, dReal lx, dReal ly, dReal lz);

		[DllImport("ode", EntryPoint = "dCreatePlane")]
		public static extern IntPtr CreatePlane(IntPtr space, dReal a, dReal b, dReal c, dReal d);

		[DllImport("ode", EntryPoint = "dGeomCopyPosition")]
		public static extern void GeomCopyPosition(IntPtr geom, out Vector3 pos);

		[DllImport("ode", EntryPoint = "dGeomDestroy")]
		public static extern void GeomDestroy(IntPtr geom);

		[DllImport("ode", EntryPoint = "dGeomGetBody")]
		public static extern IntPtr GeomGetBody(IntPtr geom);

		[DllImport("ode", EntryPoint = "dGeomSetBody")]
		public static extern void GeomSetBody(IntPtr geom, IntPtr body);

		[DllImport("ode", EntryPoint = "dHashSpaceCreate")]
		public static extern IntPtr HashSpaceCreate(IntPtr space);

		[DllImport("ode", EntryPoint = "dJointGroupCreate")]
		public static extern IntPtr JointGroupCreate(int max_size);

		[DllImport("ode", EntryPoint = "dJointGroupDestroy")]
		public static extern void JointGroupDestroy(IntPtr group);

		[DllImport("ode", EntryPoint = "dJointGroupEmpty")]
		public static extern void JointGroupEmpty(IntPtr group);

		[DllImport("ode", EntryPoint = "dRandReal")]
		public static extern dReal RandReal();

		[DllImport("ode", EntryPoint = "dRFromAxisAndAngle")]
		public static extern void RFromAxisAndAngle(out Matrix3 R, dReal x, dReal y, dReal z, dReal angle);

		[DllImport("ode", EntryPoint = "dSpaceDestroy")]
		public static extern void SpaceDestroy(IntPtr space);

		[DllImport("ode", EntryPoint = "dWorldCreate")]
		public static extern IntPtr WorldCreate();

		[DllImport("ode", EntryPoint = "dWorldDestroy")]
		public static extern void WorldDestroy(IntPtr world);

		[DllImport("ode", EntryPoint = "dWorldQuickStep")]
		public static extern void WorldQuickStep(IntPtr world, dReal stepsize);

		[DllImport("ode", EntryPoint = "dWorldSetAutoDisableFlag")]
		public static extern void WorldSetAutoDisableFlag(IntPtr world, bool do_auto_disable);

		[DllImport("ode", EntryPoint = "dWorldSetCFM")]
		public static extern void WorldSetCFM(IntPtr world, dReal cfm);

		[DllImport("ode", EntryPoint = "dWorldSetContactMaxCorrectingVel")]
		public static extern void WorldSetContactMaxCorrectingVel(IntPtr world, dReal vel);

		[DllImport("ode", EntryPoint = "dWorldSetContactSurfaceLayer")]
		public static extern void WorldSetContactSurfaceLayer(IntPtr world, dReal depth);

		[DllImport("ode", EntryPoint = "dWorldSetGravity")]
		public static extern void WorldSetGravity(IntPtr world, dReal x, dReal y, dReal z);
	}
}

