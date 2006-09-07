using System;
using System.Collections.Generic;
using Drawstuff.NET;

namespace Ode.NET
{
	public class TestBoxStack
	{
		const int NUM = 100;
		const float DENSITY = 5.0f;
		const int GPB = 3;
		const int MAX_CONTACTS = 8;
		
		static IntPtr world;
		static IntPtr space;
		static IntPtr contactgroup;

		static Queue<IntPtr> obj = new Queue<IntPtr>();

		static bool random_pos;

		static d.Vector3 xyz = new d.Vector3(2.1640f, -1.3079f, 1.7600f);
		static d.Vector3 hpr = new d.Vector3(125.5000f, -17.0000f, 0.0000f);


		static void start(int unused)
		{
			ds.SetViewpoint(ref xyz, ref hpr);
			Console.WriteLine("To drop another object, press:");
			Console.WriteLine("   b for box.");
			Console.WriteLine("   s for sphere.");
			Console.WriteLine("   c for capsule.");
			Console.WriteLine("   y for cylinder.");
			Console.WriteLine("   v for a convex object.");
			Console.WriteLine("   x for a composite object.");
			Console.WriteLine("To select an object, press space.");
			Console.WriteLine("To disable the selected object, press d.");
			Console.WriteLine("To enable the selected object, press e.");
			Console.WriteLine("To toggle showing the geom AABBs, press a.");
			Console.WriteLine("To toggle showing the contact points, press t.");
			Console.WriteLine("To toggle dropping from random position/orientation, press r.");
			Console.WriteLine("To save the current state to 'state.dif', press 1.");
		}


		static void addBody(IntPtr geom)
		{
			// Create a body for this object
			IntPtr body = d.BodyCreate(world);
			d.GeomSetBody(geom, body);
			obj.Enqueue(geom);

			// Set the position of the new object
			d.Matrix3 r;
			if (random_pos)
			{
				d.BodySetPosition(body, d.RandReal() * 2 - 1, d.RandReal() * 2 - 1, d.RandReal() + 2);
			}
			else
			{
				// Just a little higher than the highest object
				float maxheight = 0;
				foreach (IntPtr g in obj)
				{
					d.Vector3 pos;
					d.GeomCopyPosition(g, out pos);
					if (pos.Z > maxheight)
						maxheight = pos.Z;
				}
				d.BodySetPosition(body, 0, 0, maxheight + 1);
				d.RFromAxisAndAngle(out r, 0, 0, 1, 0);
			}

			// Cap the total number of objects
			if (obj.Count > NUM)
			{
				geom = obj.Dequeue();
				body = d.GeomGetBody(geom);
				d.BodyDestroy(body);
				d.GeomDestroy(geom);
			}
		}


		static void command(int cmd)
		{
			IntPtr geom;

			Char ch = Char.ToLower((Char)cmd);
			switch ((Char)ch)
			{
			case 'b':
				geom = d.CreateBox(space, 1.0f, 1.0f, 1.0f);
				addBody(geom);
				break;
			}
		}


		static void drawGeom(IntPtr geom)
		{
			IntPtr body = d.GeomGetBody(geom);

			d.Vector3 pos;
			d.BodyCopyPosition(body, out pos);

			d.Matrix3 r = new d.Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);

			d.Vector3 sides = new d.Vector3(1, 1, 1);

			ds.DrawBox(ref pos, ref r, ref sides);
		}


		static void step(int pause)
		{
			if (pause == 0)
				d.WorldQuickStep(world, 0.02f);
			d.JointGroupEmpty(contactgroup);

			ds.SetColor(1.0f, 1.0f, 0.0f);
			ds.SetTexture(ds.Texture.Wood);

			foreach (IntPtr geom in obj)
			{
				drawGeom(geom);
			}
		}


		static void Main(string[] args)
		{
			// Setup pointers to drawstuff callback functions
			ds.Functions fn;
			fn.version = ds.VERSION;
			fn.start = new ds.CallbackFunction(start);
			fn.step = new ds.CallbackFunction(step);
			fn.command = new ds.CallbackFunction(command);
			fn.stop = null;
			fn.path_to_textures = "../../drawstuff/textures";
			if (args.Length > 0)
			{
				fn.path_to_textures = args[0];
			}

			world = d.WorldCreate();
			space = d.HashSpaceCreate(IntPtr.Zero);
			contactgroup = d.JointGroupCreate(0);
			d.WorldSetGravity(world, 0.0f, 0.0f, -0.5f);
			d.WorldSetCFM(world, 1e-5f);
			d.WorldSetAutoDisableFlag(world, true);
			d.WorldSetContactMaxCorrectingVel(world, 0.1f);
			d.WorldSetContactSurfaceLayer(world, 0.001f);
			d.CreatePlane(space, 0, 0, 1, 0);

			ds.SimulationLoop(args.Length, args, 352, 288, ref fn);

			d.JointGroupDestroy(contactgroup);
			d.SpaceDestroy(space);
			d.WorldDestroy(world);
			d.CloseODE();
		}
	}
}
