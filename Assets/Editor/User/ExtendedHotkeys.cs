//Gameready assets, tips & tricks, tutorials www.not-lonely.com

//Usage:
//Put this script to Assets/Editor/User/

//Hotkeys:
//Save Assets using Shift + S

using UnityEngine;
using UnityEditor;

namespace User {
  public class ExtendedHotkeys : ScriptableObject {
    //Apply changes to prefabs while in play mode!
    [MenuItem ("User/Save Assets #_s")]
    static void DoSaveProject(){
      AssetDatabase.SaveAssets();
    }
  }
}