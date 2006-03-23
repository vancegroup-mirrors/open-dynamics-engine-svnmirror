package.name = "ode"
package.path = options["target"]
package.language = "c++"
package.objdir = "obj/ode"


-- Write a custom <config.h> to include/ode, based on the specified flags

  os.copyfile("config.in", "../include/ode/config.h")
  local f = io.open("../include/ode/config.h", "a")

  if (options["with-doubles"]) then
    f:write("#define dDOUBLE 1\n")
    f:write("#define dInfinity ODE_INFINITY8\n")
    f:write("#define dEpsilon DBL_EPSILON\n")
  else
    f:write("#define dSINGLE 1\n")
    f:write("#define dInfinity ODE_INFINITY4\n")
    f:write("#define dEpsilon FLT_EPSILON\n")
  end

  if (not options["no-cylinder"]) then
    f:write("#define dCYLINDER_ENABLED 1\n")
  end
  
  if (not options["no-trimesh"]) then
    f:write("#define dTRIMESH_ENABLED 1\n")
  end

  f:write("\n#endif\n")	
  f:close()

  
-- Package Build Settings

  if (options["enable-static"]) then
    package.kind = "lib"
  else
    package.kind = "dll"
    table.insert(package.defines, "ODE_DLL_EXPORT")
  end
  
  package.includepaths =
  {
    "../../include",
    "../../OPCODE"
  }
 
  if (windows) then
    table.insert(package.defines, "WIN32")
  end
 
 -- disable VS2005 CRT security warnings
  if (options["target"] == "vs2005") then
    table.insert(package.defines, "_CRT_SECURE_NO_DEPRECATE")
  end
  

-- Libraries

  if (windows) then
    table.insert(package.links, "user32")
  end

    
-- Files

  core_files =
  {
    matchfiles("../../include/ode/*.h"),
    matchfiles ("../../ode/src/*.h", "../../ode/src/*.c", "../../ode/src/*.cpp")
  }

  excluded_files =
  {
    "../../ode/src/scrapbook.cpp",
    "../../ode/src/stack.cpp"
  }

  trimesh_files =
  {
    "../../ode/src/collision_trimesh_internal.h",
    "../../ode/src/collision_trimesh.cpp",
    "../../ode/src/collision_trimesh_box.cpp",
    "../../ode/src/collision_trimesh_ccylinder.cpp",
    "../../ode/src/collision_cylinder_trimesh.cpp",
    "../../ode/src/collision_trimesh_distance.cpp",
    "../../ode/src/collision_trimesh_ray.cpp",
    "../../ode/src/collision_trimesh_sphere.cpp",
    "../../ode/src/collision_trimesh_trimesh.cpp"
  }
  
  opcode_files =
  {
    matchrecursive("../../OPCODE/*.h", "../../OPCODE/*.cpp")
  }
  
  cylinder_files =
  {
    "../../ode/src/collision_cylinder_box.cpp",
    "../../ode/src/collision_cylinder_plane.cpp",
    "../../ode/src/collision_cylinder_sphere.cpp",
    "../../ode/src/collision_cylinder_trimesh.cpp",
  }
   
  dif_files = 
  {
    "../../ode/src/export-dif.cpp"
  }

  package.files = { core_files } 
  package.excludes = { excluded_files }
 
  if (options["no-dif"]) then
    table.insert(package.excludes, dif_files)
  end

  if (options["no-cylinder"]) then
    table.insert(package.excludes, cylinder_files)
  end
  
  if (options["no-trimesh"]) then
    table.insert(package.excludes, trimesh_files)
  else
    table.insert(package.files, opcode_files)
  end
 