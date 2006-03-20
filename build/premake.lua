project.name = "ode"

-- Project options

  addoption("enable-static", "Build ODE as a static library instead of a DLL")
  addoption("with-doubles",  "Enables double-precision math")
  addoption("with-tests",    "Builds the test applications and DrawStuff library")
  addoption("no-cylinder",   "Disable cylinder collision geometry")
  addoption("no-dif",        "Exclude DIF (Dynamics Interchange Format) exports")
  addoption("no-trimesh",    "Exclude trimesh support")
  
  
-- Bit of a hack right of the bat: I want to put each set of project files
-- into a directory with the same name as the target toolset, so I key off
-- of options["target"]. But if I run `--clean`, there is no target and
-- Premake bombs. So make sure there is always a target specified.

  if (not options["target"]) then
    options["target"] = ""
  end

  project.path = options["target"]


-- Set the output directories

  project.config["Debug"].bindir = "../lib/debug"
  project.config["Debug"].libdir = "../lib/debug"

  project.config["Release"].bindir = "../lib/release"
  project.config["Release"].libdir = "../lib/release"
  

-- Build packages

  dopackage("ode.lua")

  if (options["with-tests"]) then
    dopackage("tests.lua")
  end
    
	
-- Since package.path is set to target, --clean doesn't work as
-- expected. Remove what I can from here.

  function doclean(cmd, arg)
    docommand(cmd, arg)
    if (options["target"] == "") then
      os.remove("../include/ode/config.h")
    end
    os.rmdir("../lib/debug")
    os.rmdir("../lib/release")
  end


-- Generate all toolsets in one go

  function domakeall(cmd, arg)
    os.execute("premake --with-tests --clean --target vs6")
    os.execute("premake --with-tests --clean --target vs2002")
    os.execute("premake --with-tests --clean --target vs2003")
    os.execute("premake --with-tests --clean --target vs2005")
    -- os.execute("premake --with-tests --clean --target gnu") -- not ready yet
  end
  