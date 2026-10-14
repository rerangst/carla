// AgExec: console lines into a running sim over loopback UDP. See AgExec.h.
#include "AgExec.h"

#include "Common/UdpSocketBuilder.h"
#include "Containers/Ticker.h"
#include "Engine/Engine.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Misc/CommandLine.h"
#include "Misc/StringOutputDevice.h"
#include "Sockets.h"
#include "SocketSubsystem.h"

DEFINE_LOG_CATEGORY_STATIC(LogAgExec, Log, All);

namespace
{
FSocket *GExecSocket = nullptr;
FTSTicker::FDelegateHandle GExecTicker;

UWorld *GameWorld()
{
  if (GEngine == nullptr)
  {
    return nullptr;
  }
  for (const FWorldContext &Ctx : GEngine->GetWorldContexts())
  {
    if (Ctx.World() != nullptr && (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE))
    {
      return Ctx.World();
    }
  }
  return nullptr;
}

/// Game-thread ticker: drain the socket, run each line, answer the sender.
bool Poll(float)
{
  ISocketSubsystem *Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
  if (GExecSocket == nullptr || Sockets == nullptr)
  {
    return false;
  }
  uint32 Pending = 0;
  for (int32 Handled = 0; Handled < 16 && GExecSocket->HasPendingData(Pending); ++Handled)
  {
    TArray<uint8> Buffer;
    Buffer.SetNumUninitialized((int32)FMath::Clamp<uint32>(Pending, 1u, 65535u) + 1);
    TSharedRef<FInternetAddr> From = Sockets->CreateInternetAddr();
    int32 Read = 0;
    if (!GExecSocket->RecvFrom(Buffer.GetData(), Buffer.Num() - 1, Read, *From) || Read <= 0)
    {
      break;
    }
    Buffer[Read] = 0;
    const FString Command = FString(UTF8_TO_TCHAR((const ANSICHAR *)Buffer.GetData())).TrimStartAndEnd();

    FStringOutputDevice Out;
    Out.SetAutoEmitLineTerminator(true);
    if (!Command.IsEmpty() && GEngine != nullptr && !GEngine->Exec(GameWorld(), *Command, Out))
    {
      Out.Logf(TEXT("AgExec: not a console command or variable: %s"), *Command);
    }
    UE_LOG(LogAgExec, Display, TEXT("AgExec: %s -> %s"), *Command, *Out.TrimEnd());

    const FTCHARToUTF8 Reply(*Out);
    int32 Sent = 0;
    GExecSocket->SendTo((const uint8 *)Reply.Get(), Reply.Length(), Sent, *From);
  }
  return true;
}
} // namespace

void AgExec::Start()
{
  int32 Port = 0;
  if (!FParse::Value(FCommandLine::Get(), TEXT("AgExecPort="), Port) || Port <= 0 || Port > 65535)
  {
    return;
  }
  GExecSocket = FUdpSocketBuilder(TEXT("AgExec"))
                    .AsNonBlocking()
                    .BoundToAddress(FIPv4Address(127, 0, 0, 1))
                    .BoundToPort((uint16)Port)
                    .WithReceiveBufferSize(1 << 16)
                    .Build();
  if (GExecSocket == nullptr)
  {
    UE_LOG(LogAgExec, Error, TEXT("AgExec: could not bind udp 127.0.0.1:%d (another sim on it?)"), Port);
    return;
  }
  GExecTicker = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateStatic(&Poll), 0.0f);
  UE_LOG(LogAgExec, Display, TEXT("AgExec: console lines on udp 127.0.0.1:%d"), Port);
}

void AgExec::Stop()
{
  if (GExecTicker.IsValid())
  {
    FTSTicker::GetCoreTicker().RemoveTicker(GExecTicker);
    GExecTicker.Reset();
  }
  if (GExecSocket != nullptr)
  {
    if (ISocketSubsystem *Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM))
    {
      Sockets->DestroySocket(GExecSocket);
    }
    GExecSocket = nullptr;
  }
}
