            NifMopp 1.0.0.2
            =================
	
	Changed multi-material shape calculation. 
			
            NifMopp 1.0.0.1
			
    This file is a helper library for accessing Havok libraries for generating 
    MOPP code from meshes.
    
    Usage
    -----
    
	  VC++
	  ----
	  
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
      
      

    
