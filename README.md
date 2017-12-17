## NifMopp 1.0.0.2

This file is a helper library for accessing Havok libraries for generating MOPP code from meshes.

### Changelog

#### 1.0.0.2

- Moved to hk660r1
- Changed multi-material shape calculation.

### Usage

```cpp
struct Vector3 { float a, b, c; }
struct Triangle { short a, b, c; }

typedef int (__stdcall * fnGenerateMoppCode)(int nVerts, Vector3 const* verts, int nTris, Triangle const *tris);
typedef int (__stdcall * fnRetrieveMoppCode)(int nBuffer, unsigned char *buffer);
typedef int (__stdcall * fnRetrieveMoppScale)(float *value);
typedef int (__stdcall * fnRetrieveMoppOrigin)(Vector3 *value);

HMODULE hMoppLib = LoadLibrary( _T("NifMopp.dll") );
if (0 != hMoppLib)
{
    fnGenerateMoppCode GenerateMoppCode = (fnGenerateMoppCode)GetProcAddress( hMoppLib, _T("GenerateMoppCode") );
    fnRetrieveMoppCode RetrieveMoppCode = (fnRetrieveMoppCode)GetProcAddress( hMoppLib, _T("RetrieveMoppCode") );
    fnRetrieveMoppScale RetrieveMoppScale = (fnRetrieveMoppScale)GetProcAddress( hMoppLib, _T("RetrieveMoppScale") );
    fnRetrieveMoppOrigin RetrieveMoppOrigin = (fnRetrieveMoppOrigin)GetProcAddress( hMoppLib, _T("RetrieveMoppOrigin") );
    if ( NULL != GenerateMoppCode  && NULL != RetrieveMoppCode 
    && NULL != RetrieveMoppScale && NULL != RetrieveMoppOrigin
        )
    {
        int len = GenerateMoppCode( nv, v, nt, t );
        if ( len > 0 )
        {
            byte* code = malloc( len );
            if ( 0 != RetrieveMoppCode( len , code ) )
            {
                float scale = 0.0f;
                Vector3 origin;
                RetrieveMoppScale( &scale );
                RetrieveMoppScale( &origin );
            }
        }
    }
    FreeLibrary( hMoppLib );
}
```

### Building

You need to define $(HKDIR) in your NifMopp.vcxproj.user file.  An example user file would be:

```xml
<?xml version="1.0" encoding="utf-8"?>
<Project ToolsVersion="15.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Debug|Win32'">
    <HKDIR>C:\Havok</HKDIR>
  </PropertyGroup>
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Release|Win32'">
    <HKDIR>C:\Havok</HKDIR>
  </PropertyGroup>
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Release DLL|Win32'">
    <HKDIR>C:\Havok</HKDIR>
  </PropertyGroup>
</Project>
```
