Unicode true
ManifestDPIAware true
RequestExecutionLevel user
SetCompressor /SOLID lzma
SetCompressorDictSize 32
SetOverwrite try

!include "MUI2.nsh"
!include "LogicLib.nsh"
!include "FileFunc.nsh"
!include "TextFunc.nsh"
!include "WordFunc.nsh"
!include "x64.nsh"

!ifdef TEST_INSTANCE
    !define APP_ID "RigidBodies-InstallerTest-${TEST_INSTANCE}"
    !define APP_NAME "Rigid Bodies Installer Test ${TEST_INSTANCE}"
    !define OUTPUT_SUFFIX "-test-${TEST_INSTANCE}"
!else
    !define APP_ID "RigidBodies"
    !define APP_NAME "Rigid Bodies"
    !define OUTPUT_SUFFIX ""
!endif
!define APP_KEY "Software\${APP_ID}"
!define UNINSTALL_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_ID}"

Name "${APP_NAME}"
OutFile "${OUTPUT_DIRECTORY}\Rigid-Bodies-${VERSION}-windows-x64${OUTPUT_SUFFIX}-setup.exe"
InstallDir "$LOCALAPPDATA\Programs\${APP_NAME}"
InstallDirRegKey HKCU "${APP_KEY}" "InstallDirectory"
Icon "${ICON_FILE}"
UninstallIcon "${ICON_FILE}"
BrandingText "Rigid Bodies ${VERSION}"
ShowInstDetails nevershow
ShowUninstDetails nevershow

VIProductVersion "${VERSION}.0"
VIAddVersionKey /LANG=1033 "ProductName" "Rigid Bodies"
VIAddVersionKey /LANG=1033 "ProductVersion" "${VERSION}"
VIAddVersionKey /LANG=1033 "FileDescription" "Rigid Bodies ${VERSION} installer"
VIAddVersionKey /LANG=1033 "FileVersion" "${VERSION}"
VIAddVersionKey /LANG=1033 "CompanyName" "${AUTHOR}"
VIAddVersionKey /LANG=1033 "Comments" "${CONTACT}"
VIAddVersionKey /LANG=1033 "LegalCopyright" "${COPYRIGHT}"

!define MUI_ICON "${ICON_FILE}"
!define MUI_UNICON "${ICON_FILE}"
!define MUI_HEADERIMAGE
!define MUI_HEADERIMAGE_RIGHT
!define MUI_HEADERIMAGE_BITMAP "${BRANDING_DIRECTORY}\header.bmp"
!define MUI_WELCOMEFINISHPAGE_BITMAP "${BRANDING_DIRECTORY}\welcome.bmp"
!define MUI_UNWELCOMEFINISHPAGE_BITMAP "${BRANDING_DIRECTORY}\welcome.bmp"
!define MUI_BGCOLOR "FAFBFD"
!define MUI_TEXTCOLOR "202632"
!define MUI_INSTFILESPAGE_COLORS "202632 FAFBFD"
!define MUI_ABORTWARNING
!define MUI_WELCOMEPAGE_TITLE "Welcome to Rigid Bodies"
!define MUI_WELCOMEPAGE_TEXT "Explore motion, build experiments and see physics in action.$\r$\n$\r$\nSetup will install version ${VERSION} for your Windows account. Updating keeps your saved scenes and preferences.$\r$\n$\r$\n${COPYRIGHT}$\r$\n${CONTACT}"
!define MUI_FINISHPAGE_TITLE "Rigid Bodies is ready"
!define MUI_FINISHPAGE_TEXT "Open the playground and choose an experiment to get started."
!define MUI_FINISHPAGE_RUN "$INSTDIR\rigid_bodies.exe"
!define MUI_FINISHPAGE_RUN_TEXT "Open Rigid Bodies"
!define MUI_FINISHPAGE_LINK "Read the distribution notes"
!define MUI_FINISHPAGE_LINK_LOCATION "$INSTDIR\docs\DISTRIBUTION.md"

!insertmacro MUI_PAGE_WELCOME
!define MUI_PAGE_CUSTOMFUNCTION_PRE DirectoryPre
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"

Var ExistingDirectory
Var ExistingVersion
Var Transaction
Var SourceRoot
Var DestinationRoot
Var FileList
Var Journal
Var Failure
Var RelativePath
Var SourcePath
Var DestinationPath
Var ListHandle
Var JournalHandle
Var ParentPath
Var Mutex
Var CheckRoot
Var CheckPath
Var FailureReason
Var RollbackNewFiles
Var KeepUninstallMetadata

; These routines never recurse through an installation directory. Every file comes from
; a package list, and each resolved path must remain below its declared root.
!macro FileOperations Prefix
Function ${Prefix}CheckPathParents
    StrCpy $4 "$CheckPath"
    ${Do}
        System::Call 'kernel32::GetFileAttributesW(w r4) i.r3'
        ${If} $3 != -1
            IntOp $3 $3 & 0x400
            ${If} $3 != 0
                StrCpy $Failure 1
                Return
            ${EndIf}
        ${EndIf}
        ${IfThen} $4 == $CheckRoot ${|} ${ExitDo} ${|}
        ${GetParent} "$4" $4
        ${IfThen} $4 == "" ${|} ${ExitDo} ${|}
    ${Loop}
FunctionEnd

Function ${Prefix}ResolveSource
    ${TrimNewLines} "$RelativePath" $RelativePath
    StrCpy $Failure 0
    System::Call 'kernel32::GetFullPathNameW(w "$SourceRoot\$RelativePath", i ${NSIS_MAX_STRLEN}, w .r5, p 0) i.r6'
    StrCpy $SourcePath $5
    StrCpy $0 "$SourceRoot\"
    StrLen $1 $0
    StrCpy $2 $SourcePath $1
    ${If} $RelativePath == ""
    ${OrIf} $2 != $0
    ${OrIf} $6 == 0
    ${OrIf} $6 >= ${NSIS_MAX_STRLEN}
        StrCpy $Failure 1
    ${EndIf}
    StrCpy $CheckRoot "$SourceRoot"
    StrCpy $CheckPath "$SourcePath"
    Call ${Prefix}CheckPathParents
FunctionEnd

Function ${Prefix}PruneParents
    ${GetParent} "$SourcePath" $ParentPath
    ${DoWhile} $ParentPath != $SourceRoot
        ${If} $ParentPath == ""
            ${ExitDo}
        ${EndIf}
        RMDir "$ParentPath"
        ${GetParent} "$ParentPath" $ParentPath
    ${Loop}
    ClearErrors
FunctionEnd

Function ${Prefix}RemoveListedFiles
    StrCpy $Failure 0
    IfFileExists "$FileList" 0 remove_done
    ClearErrors
    FileOpen $ListHandle "$FileList" r
    ${If} ${Errors}
        StrCpy $Failure 1
        Return
    ${EndIf}
    ${Do}
        ClearErrors
        FileRead $ListHandle $RelativePath
        ${IfThen} ${Errors} ${|} ${ExitDo} ${|}
        Call ${Prefix}ResolveSource
        ${IfThen} $Failure != 0 ${|} ${ExitDo} ${|}
        ${If} $KeepUninstallMetadata == "1"
            ${If} $RelativePath == "PACKAGE-FILES.txt"
            ${OrIf} $RelativePath == "Uninstall.exe"
                ${Continue}
            ${EndIf}
        ${EndIf}
        ; A write-ahead record may exist even if its subsequent rename failed.
        ${If} $RollbackNewFiles == "1"
        ${AndIf} ${FileExists} "$Transaction\stage\$RelativePath"
            ${Continue}
        ${EndIf}
        SetFileAttributes "$SourcePath" NORMAL
        Delete "$SourcePath"
        ${If} ${FileExists} "$SourcePath"
            StrCpy $Failure 1
            ${ExitDo}
        ${EndIf}
        Call ${Prefix}PruneParents
    ${Loop}
    FileClose $ListHandle
remove_done:
FunctionEnd

Function ${Prefix}CheckFilesAvailable
    StrCpy $Failure 0
    IfFileExists "$FileList" 0 check_done
    ClearErrors
    FileOpen $ListHandle "$FileList" r
    ${If} ${Errors}
        StrCpy $Failure 1
        Return
    ${EndIf}
    ${Do}
        ClearErrors
        FileRead $ListHandle $RelativePath
        ${IfThen} ${Errors} ${|} ${ExitDo} ${|}
        Call ${Prefix}ResolveSource
        ${IfThen} $Failure != 0 ${|} ${ExitDo} ${|}
        ${If} ${FileExists} "$SourcePath"
            ; Exclusive read access catches running binaries and other files held open.
            System::Call 'kernel32::CreateFileW(w "$SourcePath", i 0x80000000, i 0, p 0, i 3, i 0, p 0) p.r3'
            ${If} $3 == -1
                StrCpy $Failure 1
                ${ExitDo}
            ${EndIf}
            System::Call 'kernel32::CloseHandle(p r3)'
        ${EndIf}
    ${Loop}
    FileClose $ListHandle
check_done:
FunctionEnd
!macroend
!insertmacro FileOperations ""
!insertmacro FileOperations "un."

Function MoveListedFiles
    StrCpy $Failure 0
    IfFileExists "$FileList" 0 move_done
    ClearErrors
    FileOpen $ListHandle "$FileList" r
    ${If} ${Errors}
        StrCpy $Failure 1
        Return
    ${EndIf}
    FileOpen $JournalHandle "$Journal" a
    ${If} ${Errors}
        FileClose $ListHandle
        StrCpy $Failure 1
        Return
    ${EndIf}
    ${Do}
        ClearErrors
        FileRead $ListHandle $RelativePath
        ${IfThen} ${Errors} ${|} ${ExitDo} ${|}
        Call ResolveSource
        ${IfThen} $Failure != 0 ${|} ${ExitDo} ${|}
        ${If} ${FileExists} "$SourcePath"
            System::Call 'kernel32::GetFullPathNameW(w "$DestinationRoot\$RelativePath", i ${NSIS_MAX_STRLEN}, w .r5, p 0) i.r6'
            StrCpy $DestinationPath $5
            ${If} $6 == 0
            ${OrIf} $6 >= ${NSIS_MAX_STRLEN}
                StrCpy $Failure 1
                ${ExitDo}
            ${EndIf}
            StrCpy $CheckRoot "$DestinationRoot"
            StrCpy $CheckPath "$DestinationPath"
            Call CheckPathParents
            ${IfThen} $Failure != 0 ${|} ${ExitDo} ${|}
            ${If} ${FileExists} "$DestinationPath"
                StrCpy $Failure 1
                ${ExitDo}
            ${EndIf}
            ${GetParent} "$DestinationPath" $ParentPath
            ClearErrors
            CreateDirectory "$ParentPath"
            ; Commit the recovery record before moving the file.
            FileWrite $JournalHandle "$RelativePath$\r$\n"
            ${If} ${Errors}
                StrCpy $Failure 1
                ${ExitDo}
            ${EndIf}
            System::Call 'kernel32::FlushFileBuffers(p $JournalHandle) i.r3'
            ${If} $3 == 0
                StrCpy $Failure 1
                ${ExitDo}
            ${EndIf}
            Rename "$SourcePath" "$DestinationPath"
            ${If} ${Errors}
                StrCpy $Failure 1
                ${ExitDo}
            ${EndIf}
        ${EndIf}
    ${Loop}
    FileClose $ListHandle
    FileClose $JournalHandle
move_done:
FunctionEnd

Function CleanTransaction
    StrCpy $SourceRoot "$Transaction\stage"
    StrCpy $FileList "$Transaction\stage-files.txt"
    Call RemoveListedFiles
    ${IfThen} $Failure != 0 ${|} Return ${|}
    RMDir "$SourceRoot"
    ${If} ${FileExists} "$SourceRoot\*.*"
        StrCpy $Failure 1
        Return
    ${EndIf}
    StrCpy $SourceRoot "$Transaction\backup"
    StrCpy $FileList "$Transaction\old-files.txt"
    Call RemoveListedFiles
    ${IfThen} $Failure != 0 ${|} Return ${|}
    RMDir "$SourceRoot"
    ${If} ${FileExists} "$SourceRoot\*.*"
        StrCpy $Failure 1
        Return
    ${EndIf}
    ClearErrors
    Delete "$Transaction\stage-files.txt"
    Delete "$Transaction\old-files.txt"
    Delete "$Transaction\old-journal.txt"
    Delete "$Transaction\new-journal.txt"
    Delete "$Transaction\restore-journal.txt"
    ${If} ${Errors}
        StrCpy $Failure 1
        Return
    ${EndIf}
    Delete "$Transaction\state.ini"
    RMDir "$Transaction"
FunctionEnd

Function Rollback
    StrCpy $SourceRoot "$INSTDIR"
    StrCpy $FileList "$Transaction\new-journal.txt"
    StrCpy $RollbackNewFiles "1"
    Call RemoveListedFiles
    StrCpy $RollbackNewFiles "0"
    ${IfThen} $Failure != 0 ${|} Return ${|}
    ; Once restoration starts, retries must not remove files already restored.
    IfFileExists "$Transaction\new-journal.txt" 0 restore_old
    ClearErrors
    Delete "$Transaction\new-journal.txt"
    ${If} ${Errors}
        StrCpy $Failure 1
        Return
    ${EndIf}
restore_old:
    StrCpy $SourceRoot "$Transaction\backup"
    StrCpy $DestinationRoot "$INSTDIR"
    StrCpy $FileList "$Transaction\old-journal.txt"
    StrCpy $Journal "$Transaction\restore-journal.txt"
    Call MoveListedFiles
    ${IfThen} $Failure != 0 ${|} Return ${|}
    Call CleanTransaction
FunctionEnd

Function .onInit
!ifdef TEST_INSTANCE
    WriteINIStr "$EXEDIR\test-install-failure.ini" "Start" "Directory" "$INSTDIR"
!endif
    SetShellVarContext current
    SetRegView 64
    ${IfNot} ${RunningX64}
        MessageBox MB_OK|MB_ICONSTOP "Rigid Bodies requires 64-bit Windows." /SD IDOK
        SetErrorLevel 10
        Abort
    ${EndIf}
    System::Call 'kernel32::CreateMutexW(p 0, i 0, w "Local\${APP_ID}-Setup") p.r0 ?e'
    Pop $1
    StrCpy $Mutex $0
    ${If} $1 == 183
        MessageBox MB_OK|MB_ICONINFORMATION "Another Rigid Bodies setup is already running." /SD IDOK
        SetErrorLevel 11
        Abort
    ${EndIf}
    ReadRegStr $ExistingDirectory HKCU "${APP_KEY}" "InstallDirectory"
    ReadRegStr $ExistingVersion HKCU "${UNINSTALL_KEY}" "DisplayVersion"
    ${If} $ExistingDirectory != ""
        ; Updates always use the existing path, including silent /D= requests.
        StrCpy $INSTDIR $ExistingDirectory
        ${VersionCompare} "$ExistingVersion" "${VERSION}" $0
        ${If} $0 == 1
            MessageBox MB_OK|MB_ICONSTOP "Version $ExistingVersion is already installed. This installer contains ${VERSION}. Uninstall the newer version first if you want to go back." /SD IDOK
            SetErrorLevel 12
            Abort
        ${EndIf}
    ${EndIf}
FunctionEnd

Function DirectoryPre
    ${IfThen} $ExistingDirectory != "" ${|} Abort ${|}
FunctionEnd

Section "Rigid Bodies" SEC_MAIN
    SectionIn RO
    InitPluginsDir
    StrCpy $FailureReason "preparing the destination"
    System::Call 'kernel32::GetFullPathNameW(w "$INSTDIR", i ${NSIS_MAX_STRLEN}, w .r5, p 0) i.r6'
    ${If} $6 == 0
    ${OrIf} $6 >= ${NSIS_MAX_STRLEN}
        Goto install_failed
    ${EndIf}
    StrCpy $INSTDIR $5
    StrCpy $Transaction "$INSTDIR\.rigid-bodies-update"
    ${If} ${FileExists} "$Transaction\state.ini"
        ReadINIStr $0 "$Transaction\state.ini" "Transaction" "Owner"
        ${If} $0 != "${APP_ID}"
            Goto install_failed
        ${EndIf}
        ReadINIStr $0 "$Transaction\state.ini" "Transaction" "Committed"
        ${If} $0 == "1"
            Call CleanTransaction
        ${Else}
            Call Rollback
            ${IfThen} $Failure != 0 ${|} Goto install_failed ${|}
        ${EndIf}
    ${EndIf}
    ; An unrecognised staging directory is never overwritten or removed.
    IfFileExists "$Transaction\*.*" install_failed
    ClearErrors
    StrCpy $FailureReason "extracting the staged payload"
    CreateDirectory "$Transaction\stage"
    WriteINIStr "$Transaction\state.ini" "Transaction" "Owner" "${APP_ID}"
    File "/oname=$Transaction\stage-files.txt" "${PAYLOAD}\PACKAGE-FILES.txt"
    SetOutPath "$Transaction\stage"
    File /r "${PAYLOAD}\*"
    WriteUninstaller "$Transaction\stage\Uninstall.exe"
    ${If} ${Errors}
        Call CleanTransaction
        Goto install_failed
    ${EndIf}
    SetOutPath "$TEMP"
    ${If} $ExistingDirectory != ""
        IfFileExists "$INSTDIR\PACKAGE-FILES.txt" 0 install_failed
        ClearErrors
        CopyFiles /SILENT "$INSTDIR\PACKAGE-FILES.txt" "$Transaction\old-files.txt"
        ${If} ${Errors}
            Call CleanTransaction
            Goto install_failed
        ${EndIf}
    ${EndIf}
check_available:
    StrCpy $SourceRoot "$INSTDIR"
    StrCpy $FileList "$Transaction\old-files.txt"
    Call CheckFilesAvailable
    ${If} $Failure != 0
        MessageBox MB_RETRYCANCEL|MB_ICONEXCLAMATION "Close Rigid Bodies and any program using its installation files, then choose Retry." /SD IDCANCEL IDRETRY check_available
        Call CleanTransaction
        SetErrorLevel 13
        Abort
    ${EndIf}
    DetailPrint "Saving the installed files..."
    StrCpy $FailureReason "backing up the installed files"
    StrCpy $DestinationRoot "$Transaction\backup"
    StrCpy $Journal "$Transaction\old-journal.txt"
    Call MoveListedFiles
    ${IfThen} $Failure != 0 ${|} Goto rollback_failed_install ${|}
    DetailPrint "Installing Rigid Bodies ${VERSION}..."
    StrCpy $FailureReason "moving the new payload into place"
    StrCpy $SourceRoot "$Transaction\stage"
    StrCpy $DestinationRoot "$INSTDIR"
    StrCpy $FileList "$Transaction\stage-files.txt"
    StrCpy $Journal "$Transaction\new-journal.txt"
    Call MoveListedFiles
    ${IfThen} $Failure != 0 ${|} Goto rollback_failed_install ${|}

    ClearErrors
    CreateDirectory "$SMPROGRAMS\${APP_NAME}"
    CreateShortcut "$SMPROGRAMS\${APP_NAME}\Rigid Bodies.lnk" "$INSTDIR\rigid_bodies.exe" "" "$INSTDIR\rigid_bodies.exe" 0
    CreateShortcut "$SMPROGRAMS\${APP_NAME}\Readme.lnk" "$INSTDIR\README.md"
    CreateShortcut "$SMPROGRAMS\${APP_NAME}\Uninstall.lnk" "$INSTDIR\Uninstall.exe"
    WriteRegStr HKCU "${APP_KEY}" "InstallDirectory" "$INSTDIR"
    WriteRegStr HKCU "${UNINSTALL_KEY}" "DisplayName" "${APP_NAME}"
    WriteRegStr HKCU "${UNINSTALL_KEY}" "DisplayVersion" "${VERSION}"
    WriteRegStr HKCU "${UNINSTALL_KEY}" "Publisher" "${AUTHOR}"
    WriteRegStr HKCU "${UNINSTALL_KEY}" "Contact" "${CONTACT}"
    WriteRegStr HKCU "${UNINSTALL_KEY}" "DisplayIcon" "$INSTDIR\rigid_bodies.exe"
    WriteRegStr HKCU "${UNINSTALL_KEY}" "InstallLocation" "$INSTDIR"
    WriteRegStr HKCU "${UNINSTALL_KEY}" "UninstallString" '$\"$INSTDIR\Uninstall.exe$\"'
    WriteRegStr HKCU "${UNINSTALL_KEY}" "QuietUninstallString" '$\"$INSTDIR\Uninstall.exe$\" /S'
    WriteRegDWORD HKCU "${UNINSTALL_KEY}" "NoModify" 1
    WriteRegDWORD HKCU "${UNINSTALL_KEY}" "NoRepair" 1
    ${IfNot} ${Errors}
        WriteINIStr "$Transaction\state.ini" "Transaction" "Committed" "1"
    ${EndIf}
    ${If} ${Errors}
        ${If} $ExistingDirectory != ""
            WriteRegStr HKCU "${UNINSTALL_KEY}" "DisplayVersion" "$ExistingVersion"
        ${Else}
            DeleteRegKey HKCU "${UNINSTALL_KEY}"
            DeleteRegKey HKCU "${APP_KEY}"
            Delete "$SMPROGRAMS\${APP_NAME}\Rigid Bodies.lnk"
            Delete "$SMPROGRAMS\${APP_NAME}\Readme.lnk"
            Delete "$SMPROGRAMS\${APP_NAME}\Uninstall.lnk"
            RMDir "$SMPROGRAMS\${APP_NAME}"
        ${EndIf}
        Goto rollback_failed_install
    ${EndIf}
    Call CleanTransaction
    SetErrorLevel 0
    Goto install_done
rollback_failed_install:
    Call Rollback
install_failed:
!ifdef TEST_INSTANCE
    WriteINIStr "$EXEDIR\test-install-failure.ini" "Failure" "Action" "$FailureReason"
    WriteINIStr "$EXEDIR\test-install-failure.ini" "Failure" "Directory" "$INSTDIR"
    WriteINIStr "$EXEDIR\test-install-failure.ini" "Failure" "Source" "$SourcePath"
    WriteINIStr "$EXEDIR\test-install-failure.ini" "Failure" "Destination" "$DestinationPath"
!endif
    MessageBox MB_OK|MB_ICONSTOP "Setup could not finish updating the application files. Check the folder permissions and free disk space, close programs using this folder, then run setup again to complete recovery." /SD IDOK
    SetErrorLevel 14
    Abort
install_done:
SectionEnd

Function un.onInit
    SetShellVarContext current
    SetRegView 64
    System::Call 'kernel32::CreateMutexW(p 0, i 0, w "Local\${APP_ID}-Setup") p.r0 ?e'
    Pop $1
    StrCpy $Mutex $0
    ${If} $1 == 183
        MessageBox MB_OK|MB_ICONINFORMATION "Another Rigid Bodies setup is already running." /SD IDOK
        SetErrorLevel 11
        Abort
    ${EndIf}
FunctionEnd

Section "Uninstall"
    InitPluginsDir
    SetOutPath "$TEMP"
    ReadRegStr $ExistingDirectory HKCU "${APP_KEY}" "InstallDirectory"
    ${If} $ExistingDirectory != $INSTDIR
        MessageBox MB_OK|MB_ICONSTOP "This uninstaller does not belong to the current installation." /SD IDOK
        SetErrorLevel 15
        Abort
    ${EndIf}
    ClearErrors
    CopyFiles /SILENT "$INSTDIR\PACKAGE-FILES.txt" "$PLUGINSDIR\package-files.txt"
    ${If} ${Errors}
        SetErrorLevel 16
        Abort
    ${EndIf}
uninstall_retry:
    StrCpy $SourceRoot "$INSTDIR"
    StrCpy $FileList "$PLUGINSDIR\package-files.txt"
    Call un.CheckFilesAvailable
    ${If} $Failure != 0
        MessageBox MB_RETRYCANCEL|MB_ICONEXCLAMATION "Close Rigid Bodies and any program using its installation files, then choose Retry." /SD IDCANCEL IDRETRY uninstall_retry
        SetErrorLevel 13
        Abort
    ${EndIf}
    StrCpy $KeepUninstallMetadata "1"
    Call un.RemoveListedFiles
    ${If} $Failure != 0
        MessageBox MB_OK|MB_ICONSTOP "Some application files could not be removed. Close programs using this folder, then run the uninstaller again." /SD IDOK
        SetErrorLevel 16
        Abort
    ${EndIf}
    ClearErrors
    SetFileAttributes "$INSTDIR\Uninstall.exe" NORMAL
    Delete "$INSTDIR\Uninstall.exe"
    ${IfNot} ${Errors}
        SetFileAttributes "$INSTDIR\PACKAGE-FILES.txt" NORMAL
        Delete "$INSTDIR\PACKAGE-FILES.txt"
    ${EndIf}
    ${If} ${Errors}
        SetErrorLevel 16
        Abort
    ${EndIf}
    Delete "$SMPROGRAMS\${APP_NAME}\Rigid Bodies.lnk"
    Delete "$SMPROGRAMS\${APP_NAME}\Readme.lnk"
    Delete "$SMPROGRAMS\${APP_NAME}\Uninstall.lnk"
    RMDir "$SMPROGRAMS\${APP_NAME}"
    RMDir "$INSTDIR"
    DeleteRegKey HKCU "${UNINSTALL_KEY}"
    DeleteRegKey HKCU "${APP_KEY}"
    SetErrorLevel 0
SectionEnd
