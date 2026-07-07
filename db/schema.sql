-- ORQA cloud state schema (Supabase / Postgres)
-- Run this once in your Supabase project's SQL editor.
-- The client stores each ORQA.store key as a row; RLS ensures a user only ever
-- sees their own rows. The anon key in the client is safe by design — RLS is the guard.

create table if not exists public.app_state (
  user_id    uuid        not null references auth.users(id) on delete cascade,
  k          text        not null,
  v          text,
  updated_at timestamptz not null default now(),
  primary key (user_id, k)
);

alter table public.app_state enable row level security;

drop policy if exists "own rows read"   on public.app_state;
drop policy if exists "own rows write"  on public.app_state;
create policy "own rows read"  on public.app_state for select using (auth.uid() = user_id);
create policy "own rows write" on public.app_state for all
  using (auth.uid() = user_id) with check (auth.uid() = user_id);
