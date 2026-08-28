namespace Gravemaskin.Shell

open System
open System.IO
open System.Text.Json

type ControlPattern =
    | Iso = 0
    | Sae = 1

[<CLIMutable>]
type GameSettings =
    { ControlPattern: ControlPattern
      Volume: float32
      GamepadDeadzone: float32
      WindowWidth: int
      WindowHeight: int }

[<RequireQualifiedAccess>]
module Settings =
    let defaults =
        { ControlPattern = ControlPattern.Iso
          Volume = 0.8f
          GamepadDeadzone = 0.15f
          WindowWidth = 1440
          WindowHeight = 900 }

    let private home () =
        Environment.GetEnvironmentVariable "GRAVEMASKIN_HOME"
        |> Option.ofObj
        |> Option.defaultWith (fun () ->
            Path.Combine(
                Environment.GetFolderPath Environment.SpecialFolder.ApplicationData,
                "Gravemaskin"
            ))

    let private path () = Path.Combine(home (), "settings.json")

    /// Clamp hand-edited files back to sanity instead of crashing on them.
    let private clamp (settings: GameSettings) =
        { settings with
            Volume = Math.Clamp(settings.Volume, 0.0f, 1.0f)
            GamepadDeadzone = Math.Clamp(settings.GamepadDeadzone, 0.0f, 0.5f)
            WindowWidth = Math.Clamp(settings.WindowWidth, 640, 7680)
            WindowHeight = Math.Clamp(settings.WindowHeight, 480, 4320) }

    let load () =
        try
            if File.Exists(path ()) then
                JsonSerializer.Deserialize<GameSettings>(File.ReadAllText(path ())) |> clamp
            else
                defaults
        with _ ->
            defaults

    let save (settings: GameSettings) =
        try
            Directory.CreateDirectory(home ()) |> ignore

            File.WriteAllText(
                path (),
                JsonSerializer.Serialize(settings, JsonSerializerOptions(WriteIndented = true))
            )
        with _ ->
            ()
