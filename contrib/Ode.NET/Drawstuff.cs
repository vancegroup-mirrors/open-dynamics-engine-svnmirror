using System;
using System.Runtime.InteropServices;

namespace Drawstuff.NET
{
	public class ds
	{
		public const int VERSION = 2;

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate void CallbackFunction(int arg);

		[StructLayout(LayoutKind.Sequential)]
		public struct Functions
		{
			public int version;
			public CallbackFunction start;
			public CallbackFunction step;
			public CallbackFunction command;
			public CallbackFunction stop;
			public string path_to_textures;
		}


		[DllImport("drawstuff", EntryPoint="dsSimulationLoop")]
		public static extern void SimulationLoop(int argc, string[] argv, int window_width, int window_height, ref Functions fn);
	}
}
