import { JobsList } from "@/components/jobs-list";

export const dynamic = "force-dynamic";

export default function JobsPage() {
  return (
    <main className="mx-auto flex max-w-3xl flex-col gap-6 px-4 py-12">
      <header className="flex flex-col gap-1">
        <h1 className="font-bold text-3xl tracking-tight">My jobs</h1>
        <p className="text-muted-foreground text-sm">
          Agents you&apos;ve hired and the deliverables they produced.
        </p>
      </header>
      <JobsList />
    </main>
  );
}
