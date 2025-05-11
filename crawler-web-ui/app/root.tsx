import {
  isRouteErrorResponse,
  Link,
  Links,
  Meta,
  Outlet,
  Scripts,
  ScrollRestoration,
  useLocation,
} from "react-router";

import type { Route } from "./+types/root";
import "./app.css";
import classNames from "classnames";
import { useEffect, useState } from "react";
import { API } from "./api/api";

export const links: Route.LinksFunction = () => [
  { rel: "preconnect", href: "https://fonts.googleapis.com" },
  {
    rel: "preconnect",
    href: "https://fonts.gstatic.com",
    crossOrigin: "anonymous",
  },
  {
    rel: "stylesheet",
    href: "https://fonts.googleapis.com/css2?family=Inter:ital,opsz,wght@0,14..32,100..900;1,14..32,100..900&display=swap",
  },
];

export function Layout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <head>
        <meta charSet="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <Meta />
        <Links />
      </head>
      <body>
        {children}
        <ScrollRestoration />
        <Scripts />
      </body>
    </html>
  );
}

function TopNavigation() {
  const navbarRoutes = [
    {
      title: "Home",
      path: "/",
    },
    {
      title: "Manual",
      path: "/manual-control",
    },
    {
      title: "RL",
      path: "/rl-control",
    },
  ] satisfies { title: string, path: string }[];

  const location = useLocation();

  const navigationItems = (hideOnSmall: boolean) => <>
    {navbarRoutes.map((route) => (
      <li key={route.path} className={classNames(
        "px-3 py-1.5 flex items-center opacity-40 transition duration-100",
        { "opacity-100": location.pathname === route.path },
        { "hidden sm:flex": hideOnSmall },
      )}>
        <Link to={route.path}>{route.title}</Link>
      </li>
    ))}
    <li className="grow"></li>
    <li className={classNames(
      "px-3 py-1.5 flex items-center",
      { "hidden sm:flex": hideOnSmall },
    )}>
      <a href="https://github.com/hector-crawler/crawler-controller" target="_blank">GitHub</a>
    </li>
  </>;

  const [expanded, setExpanded] = useState(false);

  return (
    <nav className="bg-gray-900 text-white p-3">
      <ul className="flex gap-2">
        <li>
          <Link to="/" className="px-3 py-1.5 pr-5 font-bold flex items-center gap-4">
            <img src="/favicon.ico" />
            Crawler
          </Link>
        </li>
        {navigationItems(true)}
        <li className="px-3 py-1.5 sm:hidden flex items-center" onClick={() => setExpanded(expanded => !expanded)}>&#9776;</li>
      </ul>
      {expanded && (
        <ul className="flex gap-2 mt-3 sm:hidden">
          {navigationItems(false)}
        </ul>
      )}
    </nav>
  );
}

function Footer() {
  const [buildMetadata, setBuildMetadata] = useState("...");
  useEffect(() => {
    new API().getBuildMetadata().then(setBuildMetadata);
  }, []);

  return (
    <div className="fixed bottom-0 right-0 bg-gray-900 px-1.5 rounded-tl-lg text-gray-500 text-sm border-t-1 border-l-1 border-gray-800">
      {buildMetadata}
    </div>
  );
}

export default function App() {
  return <>
    <TopNavigation />
    <Outlet />
    <Footer />
  </>;
}

export function ErrorBoundary({ error }: Route.ErrorBoundaryProps) {
  let message = "Oops!";
  let details = "An unexpected error occurred.";
  let stack: string | undefined;

  if (isRouteErrorResponse(error)) {
    message = error.status === 404 ? "404" : "Error";
    details =
      error.status === 404
        ? "The requested page could not be found."
        : error.statusText || details;
  } else if (import.meta.env.DEV && error && error instanceof Error) {
    details = error.message;
    stack = error.stack;
  }

  return (
    <main className="pt-16 p-4 container mx-auto">
      <h1>{message}</h1>
      <p>{details}</p>
      {stack && (
        <pre className="w-full p-4 overflow-x-auto">
          <code>{stack}</code>
        </pre>
      )}
    </main>
  );
}
