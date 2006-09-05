using System;
using Drawstuff.NET;

namespace Ode.NET
{
	public class TestBoxStack
	{
		static void start(int unused)
		{
		}


		static void step(int pause)
		{
		}


		static void command(int cmd)
		{
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

			ds.SimulationLoop(args.Length, args, 352, 288, ref fn);
		}
	}
}
